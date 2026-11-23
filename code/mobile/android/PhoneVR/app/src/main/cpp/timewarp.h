#ifndef PHONEVR_TIMEWARP_H
#define PHONEVR_TIMEWARP_H

// Rotational timewarp (reprojection) for the Cardboard path, which has none of its own.
//
// Each streamed frame was rendered on the PC for a predicted head orientation. Right before it
// goes to the lens-distortion pass, each eye image is re-sampled for the orientation the head will
// have when the photons leave the screen: for every output pixel, take its view direction in the
// display-time eye frame, rotate it into the render-time eye frame, and sample the stream texture
// there. Display refreshes with no new stream frame re-warp the previous frame, so every refresh
// shows a correctly placed image instead of a stale buffer. Rotation only; translation is not
// corrected (it is small over one frame).

#include "alvr_client_core.h"
#include <GLES3/gl3.h>
#include <android/log.h>
#include <cmath>

namespace timewarp {

static const char *VERTEX_SHADER = R"(#version 300 es
out vec2 uv;
void main() {
    // One triangle covering the viewport.
    vec2 p = vec2(gl_VertexID == 1 ? 3.0 : -1.0, gl_VertexID == 2 ? 3.0 : -1.0);
    uv = p * 0.5 + 0.5;
    gl_Position = vec4(p, 0.0, 1.0);
}
)";

static const char *FRAGMENT_SHADER = R"(#version 300 es
precision highp float;
in vec2 uv;
out vec4 color;
uniform sampler2D tex;
// Maps a view direction in the display-time eye frame to the render-time eye frame.
uniform mat3 displayToRender;
// Tangents of the output (display) eye FOV: left (negative), right, down (negative), up.
uniform vec4 tanFov;
// Tangents of the FOV the PC rendered the source image with (wider with overscan).
uniform vec4 srcTanFov;
// 0: black outside the rendered image. 1: repeat its edge pixels (much less visible).
uniform int edgeMode;
// Positional timewarp: where the eye is now, in the render eye's frame (metres), and the
// distance of the assumed scene plane (0 = rotation only). See stream_tuning.h::planarReproject.
uniform vec3 eyeOffset;
uniform float planeDist;
void main() {
    vec3 d = vec3(mix(tanFov.x, tanFov.y, uv.x), mix(tanFov.z, tanFov.w, uv.y), -1.0);
    vec3 r = displayToRender * d;
    if (r.z > -1e-4) {
        color = vec4(0.0, 0.0, 0.0, 1.0);
        return;
    }
    vec2 t;
    if (planeDist > 0.0) {
        float s = (-planeDist - eyeOffset.z) / r.z;
        vec3 q = eyeOffset + s * r;
        if (s <= 0.0 || q.z > -1e-3) {
            color = vec4(0.0, 0.0, 0.0, 1.0);
            return;
        }
        t = q.xy / -q.z;
    } else {
        t = r.xy / -r.z;
    }
    vec2 src = vec2((t.x - srcTanFov.x) / (srcTanFov.y - srcTanFov.x),
                    (t.y - srcTanFov.z) / (srcTanFov.w - srcTanFov.z));
    if (any(lessThan(src, vec2(0.0))) || any(greaterThan(src, vec2(1.0)))) {
        if (edgeMode == 0) {
            color = vec4(0.0, 0.0, 0.0, 1.0);   // outside what the PC rendered
            return;
        }
        src = clamp(src, vec2(0.0), vec2(1.0));
    }
    color = texture(tex, src);
}
)";

// Column-major 3x3 rotation matrix of a unit quaternion.
inline void quatToMat3(AlvrQuat q, float m[9]) {
    float x = q.x, y = q.y, z = q.z, w = q.w;
    m[0] = 1 - 2 * (y * y + z * z);
    m[1] = 2 * (x * y + z * w);
    m[2] = 2 * (x * z - y * w);
    m[3] = 2 * (x * y - z * w);
    m[4] = 1 - 2 * (x * x + z * z);
    m[5] = 2 * (y * z + x * w);
    m[6] = 2 * (x * z + y * w);
    m[7] = 2 * (y * z - x * w);
    m[8] = 1 - 2 * (x * x + y * y);
}

class Timewarp {
  public:
    // GL context lost: every handle is already invalid, just forget them.
    void forgetGlObjects() {
        program_ = 0;
        vao_ = 0;
        for (int e = 0; e < 2; e++) {
            textures_[e] = 0;
            framebuffers_[e] = 0;
        }
        width_ = height_ = 0;
    }

    // Allocates the program and per-eye targets for width x height eye images. Render thread.
    bool create(int width, int height) {
        destroy();
        program_ = buildProgram();
        if (!program_)
            return false;
        uTex_ = glGetUniformLocation(program_, "tex");
        uMatrix_ = glGetUniformLocation(program_, "displayToRender");
        uTanFov_ = glGetUniformLocation(program_, "tanFov");
        uSrcTanFov_ = glGetUniformLocation(program_, "srcTanFov");
        uEdgeMode_ = glGetUniformLocation(program_, "edgeMode");
        uEyeOffset_ = glGetUniformLocation(program_, "eyeOffset");
        uPlaneDist_ = glGetUniformLocation(program_, "planeDist");
        glGenVertexArrays(1, &vao_);

        glGenTextures(2, textures_);
        glGenFramebuffers(2, framebuffers_);
        for (int e = 0; e < 2; e++) {
            glBindTexture(GL_TEXTURE_2D, textures_[e]);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexImage2D(
                GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
            glBindFramebuffer(GL_FRAMEBUFFER, framebuffers_[e]);
            glFramebufferTexture2D(
                GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, textures_[e], 0);
            if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
                __android_log_print(ANDROID_LOG_ERROR, "PVR-Timewarp", "framebuffer incomplete");
                glBindFramebuffer(GL_FRAMEBUFFER, 0);
                destroy();
                return false;
            }
        }
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glBindTexture(GL_TEXTURE_2D, 0);
        width_ = width;
        height_ = height;
        return true;
    }

    void destroy() {
        if (program_)
            glDeleteProgram(program_);
        if (vao_)
            glDeleteVertexArrays(1, &vao_);
        if (textures_[0])
            glDeleteTextures(2, textures_);
        if (framebuffers_[0])
            glDeleteFramebuffers(2, framebuffers_);
        forgetGlObjects();
    }

    bool ready() const { return program_ != 0 && width_ > 0; }

    // Re-samples one eye image rendered at renderOrientation (world from eye) with sourceFov for
    // displayOrientation and displayFov, into this eye's output texture. Returns that texture.
    // eyeOffset (render-eye frame, metres) + planeDistM > 0 add positional reprojection.
    GLuint warp(int eye,
                GLuint sourceTexture,
                AlvrFov sourceFov,
                AlvrFov displayFov,
                AlvrQuat renderOrientation,
                AlvrQuat displayOrientation,
                bool stretchEdges,
                const float eyeOffset[3] = nullptr,
                float planeDistM = 0) {
        // displayToRender = R(render)^-1 * R(display)
        AlvrQuat r = renderOrientation, d = displayOrientation;
        AlvrQuat inv = {-r.x, -r.y, -r.z, r.w};
        AlvrQuat q = {inv.w * d.x + inv.x * d.w + inv.y * d.z - inv.z * d.y,
                      inv.w * d.y - inv.x * d.z + inv.y * d.w + inv.z * d.x,
                      inv.w * d.z + inv.x * d.y - inv.y * d.x + inv.z * d.w,
                      inv.w * d.w - inv.x * d.x - inv.y * d.y - inv.z * d.z};
        float m[9];
        quatToMat3(q, m);

        glBindFramebuffer(GL_FRAMEBUFFER, framebuffers_[eye]);
        glViewport(0, 0, width_, height_);
        glDisable(GL_DEPTH_TEST);
        glDisable(GL_BLEND);
        glDisable(GL_SCISSOR_TEST);
        glDisable(GL_CULL_FACE);
        glDepthMask(GL_FALSE);
        glUseProgram(program_);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, sourceTexture);
        glUniform1i(uTex_, 0);
        glUniformMatrix3fv(uMatrix_, 1, GL_FALSE, m);
        glUniform4f(uTanFov_,
                    std::tan(displayFov.left),
                    std::tan(displayFov.right),
                    std::tan(displayFov.down),
                    std::tan(displayFov.up));
        glUniform4f(uSrcTanFov_,
                    std::tan(sourceFov.left),
                    std::tan(sourceFov.right),
                    std::tan(sourceFov.down),
                    std::tan(sourceFov.up));
        glUniform1i(uEdgeMode_, stretchEdges ? 1 : 0);
        if (eyeOffset && planeDistM > 0) {
            glUniform3f(uEyeOffset_, eyeOffset[0], eyeOffset[1], eyeOffset[2]);
            glUniform1f(uPlaneDist_, planeDistM);
        } else {
            glUniform3f(uEyeOffset_, 0, 0, 0);
            glUniform1f(uPlaneDist_, 0);
        }
        glBindVertexArray(vao_);
        glDrawArrays(GL_TRIANGLES, 0, 3);
        glBindVertexArray(0);
        glUseProgram(0);
        glBindTexture(GL_TEXTURE_2D, 0);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glDepthMask(GL_TRUE);
        return textures_[eye];
    }

  private:
    static GLuint compile(GLenum type, const char *src) {
        GLuint s = glCreateShader(type);
        glShaderSource(s, 1, &src, nullptr);
        glCompileShader(s);
        GLint ok = 0;
        glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
        if (!ok) {
            char log[1024];
            glGetShaderInfoLog(s, sizeof(log), nullptr, log);
            __android_log_print(ANDROID_LOG_ERROR, "PVR-Timewarp", "shader compile: %s", log);
            glDeleteShader(s);
            return 0;
        }
        return s;
    }

    static GLuint buildProgram() {
        GLuint vs = compile(GL_VERTEX_SHADER, VERTEX_SHADER);
        GLuint fs = compile(GL_FRAGMENT_SHADER, FRAGMENT_SHADER);
        if (!vs || !fs) {
            if (vs)
                glDeleteShader(vs);
            if (fs)
                glDeleteShader(fs);
            return 0;
        }
        GLuint p = glCreateProgram();
        glAttachShader(p, vs);
        glAttachShader(p, fs);
        glLinkProgram(p);
        glDeleteShader(vs);
        glDeleteShader(fs);
        GLint ok = 0;
        glGetProgramiv(p, GL_LINK_STATUS, &ok);
        if (!ok) {
            char log[1024];
            glGetProgramInfoLog(p, sizeof(log), nullptr, log);
            __android_log_print(ANDROID_LOG_ERROR, "PVR-Timewarp", "program link: %s", log);
            glDeleteProgram(p);
            return 0;
        }
        return p;
    }

    GLuint program_ = 0;
    GLuint vao_ = 0;
    GLint uTex_ = -1, uMatrix_ = -1, uTanFov_ = -1, uSrcTanFov_ = -1, uEdgeMode_ = -1;
    GLint uEyeOffset_ = -1, uPlaneDist_ = -1;
    GLuint textures_[2] = {0, 0};
    GLuint framebuffers_[2] = {0, 0};
    int width_ = 0, height_ = 0;
};

}   // namespace timewarp

#endif   // PHONEVR_TIMEWARP_H
