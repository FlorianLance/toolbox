
#pragma once

// local
#include "texture_buffer_object.hpp"
#include "render_buffer_object.hpp"

namespace tool::gl{

using namespace std::literals::string_view_literals;

enum class FrameBufferAttachment : unsigned int {
    none    = GL_NONE,
    depth   = GL_DEPTH_ATTACHMENT,
    stencil = GL_STENCIL_ATTACHMENT,
    color0  = GL_COLOR_ATTACHMENT0,
    color1  = GL_COLOR_ATTACHMENT1,
    color2  = GL_COLOR_ATTACHMENT2,
    color3  = GL_COLOR_ATTACHMENT3,
    color4  = GL_COLOR_ATTACHMENT4,
    color5  = GL_COLOR_ATTACHMENT5,
    color6  = GL_COLOR_ATTACHMENT6,
    color7  = GL_COLOR_ATTACHMENT7,
    color8  = GL_COLOR_ATTACHMENT8,
    color9  = GL_COLOR_ATTACHMENT9,
    color10 = GL_COLOR_ATTACHMENT10,
    color11 = GL_COLOR_ATTACHMENT11,
    color12 = GL_COLOR_ATTACHMENT12,
    color13 = GL_COLOR_ATTACHMENT13,
    color14 = GL_COLOR_ATTACHMENT14,
    color15 = GL_COLOR_ATTACHMENT15,
    SizeEnum
};

using IdColor = int;
using TFBAGl = std::tuple<FrameBufferAttachment, IdColor, std::string_view>;
static constexpr TupleArray<FrameBufferAttachment::SizeEnum, TFBAGl> frameBufferAttachmentsGl ={{
    TFBAGl{FrameBufferAttachment::none,    -1, "none"sv},
    TFBAGl{FrameBufferAttachment::depth,   -1, "depth"sv},
    TFBAGl{FrameBufferAttachment::stencil, -1, "stencil"sv},
    TFBAGl{FrameBufferAttachment::color0,  0, "color0"sv},
    TFBAGl{FrameBufferAttachment::color1,  1, "color1"sv},
    TFBAGl{FrameBufferAttachment::color2,  2, "color2"sv},
    TFBAGl{FrameBufferAttachment::color3,  3, "color3"sv},
    TFBAGl{FrameBufferAttachment::color4,  4, "color4"sv},
    TFBAGl{FrameBufferAttachment::color5,  5, "color5"sv},
    TFBAGl{FrameBufferAttachment::color6,  6, "color6"sv},
    TFBAGl{FrameBufferAttachment::color7,  7, "color7"sv},
    TFBAGl{FrameBufferAttachment::color8,  8, "color8"sv},
    TFBAGl{FrameBufferAttachment::color9,  9, "color9"sv},
    TFBAGl{FrameBufferAttachment::color10, 10, "color10"sv},
    TFBAGl{FrameBufferAttachment::color11, 11, "color11"sv},
    TFBAGl{FrameBufferAttachment::color12, 12, "color12"sv},
    TFBAGl{FrameBufferAttachment::color13, 13, "color13"sv},
    TFBAGl{FrameBufferAttachment::color14, 14, "color14"sv},
    TFBAGl{FrameBufferAttachment::color15, 15, "color15"sv},
}};

[[maybe_unused]] static constexpr std::string_view to_string(FrameBufferAttachment t) {
    return frameBufferAttachmentsGl.at<0,2>(t);
}

[[maybe_unused]] static constexpr FrameBufferAttachment get_color_attachment(IdColor id) {
    return frameBufferAttachmentsGl.at<1,0>(id);
}

struct FBO{

    FBO() = default;
    FBO(const FBO&) = delete;
    FBO& operator=(const FBO&) = delete;
    FBO(FBO&& other) = default;
    FBO& operator=(FBO&& other) = default;


    ~FBO(){
        clean();
    }

    void generate(){

        if(m_id != 0){
            std::cerr << "FBO already generated: " << m_id << "\n";
            return;
        }
        glCreateFramebuffers(1, &m_id);

        GLint maxAttach = 0;
        glGetIntegerv(GL_MAX_COLOR_ATTACHMENTS, &maxAttach);
        colorAttachments.resize(maxAttach);
    }

    void bind() const{
        glBindFramebuffer(GL_FRAMEBUFFER, m_id);
    }

    static void bind(const FBO &fbo){
        fbo.bind();
    }

    static void unbind(){
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }


    void attach_depth_texture(const TBO &tbo, GLint mipmapLevel = 0){
        glNamedFramebufferTexture(
            m_id,
            GL_DEPTH_ATTACHMENT,
            // attachment: attachment specifies the logical attachment of the framebuffer and must be
            //      GL_COLOR_ATTACHMENTi, GL_DEPTH_ATTACHMENT, GL_STENCIL_ATTACHMENT or GL_DEPTH_STENCIL_ATTACHMENT
            //      GL_COLOR_ATTACHMENTi may range from zero to the value of GL_MAX_COLOR_ATTACHMENTS minus one.
            //      Attaching a level of a texture to GL_DEPTH_STENCIL_ATTACHMENT is equivalent to attaching that
            //      level to both the GL_DEPTH_ATTACHMENT and the GL_STENCIL_ATTACHMENT attachment points simultaneously.
            m_depthAttachment = tbo.id(), // texture: Specifies the name of an existing texture object to attach.
            mipmapLevel   // level: Specifies the mipmap level of the texture object to attach.
        );
    }

    void attach_color0_texture(const TBO &tbo, GLint mipmapLevel = 0){

        // attach a level of a texture object as a logical buffer of a framebuffer object
        glNamedFramebufferTexture(
            m_id,
            GL_COLOR_ATTACHMENT0,
                // attachment: attachment specifies the logical attachment of the framebuffer and must be
                //      GL_COLOR_ATTACHMENTi, GL_DEPTH_ATTACHMENT, GL_STENCIL_ATTACHMENT or GL_DEPTH_STENCIL_ATTACHMENT
                //      GL_COLOR_ATTACHMENTi may range from zero to the value of GL_MAX_COLOR_ATTACHMENTS minus one.
                //      Attaching a level of a texture to GL_DEPTH_STENCIL_ATTACHMENT is equivalent to attaching that
                //      level to both the GL_DEPTH_ATTACHMENT and the GL_STENCIL_ATTACHMENT attachment points simultaneously.
            colorAttachments[0] = tbo.id(), // texture: Specifies the name of an existing texture object to attach.
            mipmapLevel   // level: Specifies the mipmap level of the texture object to attach.
        );
    }

    void attach_color1_texture(const TBO &tbo, GLint mipmapLevel = 0){

        glNamedFramebufferTexture(
            m_id,
            GL_COLOR_ATTACHMENT1,
            colorAttachments[1] = tbo.id(),
            mipmapLevel
        );
    }

    void attach_color2_texture(const TBO &tbo, GLint mipmapLevel = 0){

        glNamedFramebufferTexture(
            m_id,
            GL_COLOR_ATTACHMENT2,
            colorAttachments[2] = tbo.id(),
            mipmapLevel
        );
    }

    void attach_color3_texture(const TBO &tbo, GLint mipmapLevel = 0){

        glNamedFramebufferTexture(
            m_id,
            GL_COLOR_ATTACHMENT3,
            colorAttachments[3] = tbo.id(),
            mipmapLevel
        );
    }

    void attach_color_texture(const TBO &tbo, size_t id, GLint mipmapLevel = 0){

        if(id >= colorAttachments.size()){
            std::cerr << "Id color texture attachment too large\n.";
            return;
        }

        glNamedFramebufferTexture(
            m_id,
            static_cast<GLenum >(get_color_attachment(static_cast<int>(id))),
            colorAttachments[id] = tbo.id(),
            mipmapLevel
        );
    }

    void attach_colors_textures(const std_v1<TBO*> tboS){
        for(size_t ii = 0; ii < tboS.size(); ++ii){
            attach_color_texture(*tboS[ii], ii);
        }
    }

    void attach_depth_buffer(const RBO &rbo){

        // attach a renderbuffer as a logical buffer of a framebuffer object
        glNamedFramebufferRenderbuffer(
            m_id,
            GL_DEPTH_ATTACHMENT,
            GL_RENDERBUFFER,
            m_depthAttachment = rbo.id()
        );
    }

    void clean(){

        if(m_id == 0){
            return;
        }
        glDeleteBuffers(1, &m_id);
        m_id = 0;
    }

    bool set_draw_buffers(std_v1<gl::FrameBufferAttachment> attachments){

        // glDrawBuffers, glNamedFramebufferDrawBuffers — Specifies a list of color buffers to be drawn into

        // Set the targets for the fragment output variables
        glNamedFramebufferDrawBuffers(m_id, static_cast<GLsizei>(attachments.size()), reinterpret_cast<unsigned int*>(&attachments[0]));        

        GLenum result = glCheckNamedFramebufferStatus(m_id, GL_FRAMEBUFFER);
        if( result == GL_FRAMEBUFFER_COMPLETE) {
            // std::cout << "Framebuffer is complete\n";
            return true;
        } else {
            std::cerr << "Framebuffer error: " << result << "\n";
            return false;
        }
    }

//    bool set_read_buffer(){

//        // glReadBuffer, glNamedFramebufferReadBuffer — select a color buffer source for pixels

//        // Set the targets for the fragment output variables
//        glNamedFramebufferReadBuffer(m_id, static_cast<GLsizei>(attachments.size()), reinterpret_cast<unsigned int*>(&attachments[0]));

//        //specifies a color buffer. Accepted values are GL_FRONT_LEFT, GL_FRONT_RIGHT, GL_BACK_LEFT, GL_BACK_RIGHT, GL_FRONT, GL_BACK, GL_LEFT, GL_RIGHT, and the constants GL_COLOR_ATTACHMENTi.

//        GLenum result = glCheckNamedFramebufferStatus(m_id, GL_FRAMEBUFFER);
//        if( result == GL_FRAMEBUFFER_COMPLETE) {
//            // std::cout << "Framebuffer is complete\n";
//            return true;
//        } else {
//            std::cerr << "Framebuffer error: " << result << "\n";
//            return false;
//        }
//    }

    inline GLuint id() const{return m_id;}
    inline GLuint color_attachment(size_t id = 0) const {return colorAttachments[id];}
    inline GLuint depth_attachment() const{return m_depthAttachment;}

private:

    GLuint m_id = 0;
    std::vector<GLuint> colorAttachments = {0};
    GLuint m_depthAttachment = 0;
};
}


