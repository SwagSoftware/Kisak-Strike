#include "rocketrender.h"

#if defined RMLUI_PLATFORM_WIN32
#include <win32/IncludeWindows.h>
#include <gl/Gl.h>
#include <gl/Glu.h>
#elif defined RMLUI_PLATFORM_MACOSX
#include <AGL/agl.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <OpenGL/glext.h>
#elif defined RMLUI_PLATFORM_UNIX
#include <GL/glx.h>
#include <GL/gl.h>
#include <GL/glext.h>
#include <GL/glu.h>
// The None define from X.h conflicts with RmlUi code base,
// use the constant 0L instead where necessary
#ifdef None
#undef None
#endif
#endif

#include <RmlUi/Core.h>

RocketRender RocketRender::m_Instance;

RocketRender::RocketRender() { }

void RocketRender::PrepareGLState()
{
    glDisable(GL_CULL_FACE);

    //make sure to set both of these to zero otherwise mesa will segfault even though it only mentions GL_ARRAY_BUFFER in the docs
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);



    //int backup0, backup1, backup2, backup3, backup4, backup5, backup6, backup7, backup8, backup9, backup10,
    //        backup11, backup12, backup13, backup14, backup15;
    //glGetVertexAttribIiv(0, GL_VERTEX_ATTRIB_ARRAY_ENABLED, &backup0 );
    //glGetVertexAttribIiv(1, GL_VERTEX_ATTRIB_ARRAY_ENABLED, &backup1 );
    //glGetVertexAttribIiv(2, GL_VERTEX_ATTRIB_ARRAY_ENABLED, &backup2 );
    //glGetVertexAttribIiv(3, GL_VERTEX_ATTRIB_ARRAY_ENABLED, &backup3 );
    //glGetVertexAttribIiv(4, GL_VERTEX_ATTRIB_ARRAY_ENABLED, &backup4 );
    //glGetVertexAttribIiv(5, GL_VERTEX_ATTRIB_ARRAY_ENABLED, &backup5 );
    //glGetVertexAttribIiv(6, GL_VERTEX_ATTRIB_ARRAY_ENABLED, &backup6 );
    //glGetVertexAttribIiv(7, GL_VERTEX_ATTRIB_ARRAY_ENABLED, &backup7 );
    //glGetVertexAttribIiv(8, GL_VERTEX_ATTRIB_ARRAY_ENABLED, &backup8 );
    //glGetVertexAttribIiv(9, GL_VERTEX_ATTRIB_ARRAY_ENABLED, &backup9 );
    //glGetVertexAttribIiv(10, GL_VERTEX_ATTRIB_ARRAY_ENABLED, &backup10 );
    //glGetVertexAttribIiv(11, GL_VERTEX_ATTRIB_ARRAY_ENABLED, &backup11 );
    //glGetVertexAttribIiv(12, GL_VERTEX_ATTRIB_ARRAY_ENABLED, &backup12 );
    //glGetVertexAttribIiv(13, GL_VERTEX_ATTRIB_ARRAY_ENABLED, &backup13 );
    //glGetVertexAttribIiv(14, GL_VERTEX_ATTRIB_ARRAY_ENABLED, &backup14 );
    //glGetVertexAttribIiv(15, GL_VERTEX_ATTRIB_ARRAY_ENABLED, &backup15 );
    glDisableVertexAttribArray(0);
    glDisableVertexAttribArray(1);
    glDisableVertexAttribArray(2);
    glDisableVertexAttribArray(3);
    glDisableVertexAttribArray(4);
    glDisableVertexAttribArray(5);
    glDisableVertexAttribArray(6);
    glDisableVertexAttribArray(7);
    glDisableVertexAttribArray(8);
    glDisableVertexAttribArray(9);
    glDisableVertexAttribArray(10);
    glDisableVertexAttribArray(11);
    glDisableVertexAttribArray(12);
    glDisableVertexAttribArray(13);
    glDisableVertexAttribArray(14);
    glDisableVertexAttribArray(15);

    glDisable(GL_ALPHA_TEST);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_FALSE);

    glEnable(GL_BLEND);
    glBlendColor(1, 1, 1, 1);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glBlendEquation(GL_FUNC_ADD);

    //glStencilFunc( GL_GEQUAL, 253, -1 );
    //glAlphaFunc(GL_GEQUAL, 0);

}

void RocketRender::RenderGeometry( Rml::Vertex *vertices, int num_vertices, int *indices, int num_indices,
                                   Rml::TextureHandle texture, const Rml::Vector2f &translation )
{
    RMLUI_UNUSED(num_vertices);
    glPushMatrix();

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, m_width, 0, m_height, -10000, 10000);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);

    glTranslatef(translation.x, translation.y, 0);

    glVertexPointer(2, GL_FLOAT, sizeof(Rml::Vertex), &vertices[0].position);
    glColorPointer(4, GL_UNSIGNED_BYTE, sizeof(Rml::Vertex), &vertices[0].colour);

    if (!texture)
    {
        glDisable(GL_TEXTURE_2D);
        glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    }
    else
    {
        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, (GLuint) texture);
        glEnableClientState(GL_TEXTURE_COORD_ARRAY);
        glTexCoordPointer(2, GL_FLOAT, sizeof(Rml::Vertex), &vertices[0].tex_coord);
    }

    glDrawElements(GL_TRIANGLES, num_indices, GL_UNSIGNED_INT, indices);

    glPopMatrix();
}


Rml::CompiledGeometryHandle RocketRender::CompileGeometry(Rml::Vertex *vertices, int num_vertices, int *indices, int num_indices, Rml::TextureHandle texture)
{
    return 0;
}

// Called by RmlUi when it wants to enable or disable scissoring to clip content.
void RocketRender::EnableScissorRegion(bool enable)
{
    if (enable) {
        if (!m_transformEnabled) {
            glEnable(GL_SCISSOR_TEST);
            glDisable(GL_STENCIL_TEST);
        } else {
            glDisable(GL_SCISSOR_TEST);
            glEnable(GL_STENCIL_TEST);
        }
    } else {
        glDisable(GL_SCISSOR_TEST);
        glDisable(GL_STENCIL_TEST);
    }
}

// Called by RmlUi when it wants to change the scissor region.
void RocketRender::SetScissorRegion(int x, int y, int width, int height)
{
    if (!m_transformEnabled) {
        glScissor(x, y, width, height);
    } else {
        // clear the stencil buffer
        glStencilMask(GLuint(-1));
        glClear(GL_STENCIL_BUFFER_BIT);

        // fill the stencil buffer
        glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
        glDepthMask(GL_FALSE);
        glStencilFunc(GL_NEVER, 1, GLuint(-1));
        glStencilOp(GL_REPLACE, GL_KEEP, GL_KEEP);

        float fx = (float)x;
        float fy = (float)y;
        float fwidth = (float)width;
        float fheight = (float)height;

        // draw transformed quad
        GLfloat vertices[] = {
                fx, fy, 0,
                fx, fy + fheight, 0,
                fx + fwidth, fy + fheight, 0,
                fx + fwidth, fy, 0
        };
        glDisableClientState(GL_COLOR_ARRAY);
        glVertexPointer(3, GL_FLOAT, 0, vertices);
        GLushort indices[] = { 1, 2, 0, 3 };
        glDrawElements(GL_TRIANGLE_STRIP, 4, GL_UNSIGNED_SHORT, indices);
        glEnableClientState(GL_COLOR_ARRAY);

        // prepare for drawing the real thing
        glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
        glDepthMask(GL_TRUE);
        glStencilMask(0);
        glStencilFunc(GL_EQUAL, 1, GLuint(-1));
    }
}

// Set to byte packing, or the compiler will expand our struct, which means it won't read correctly from file
#pragma pack(1)
struct TGAHeader
{
    char  idLength;
    char  colourMapType;
    char  dataType;
    short int colourMapOrigin;
    short int colourMapLength;
    char  colourMapDepth;
    short int xOrigin;
    short int yOrigin;
    short int width;
    short int height;
    char  bitsPerPixel;
    char  imageDescriptor;
};
// Restore packing
#pragma pack()

bool RocketRender::LoadTexture(Rml::TextureHandle &texture_handle, Rml::Vector2i &texture_dimensions, const Rml::String &source)
{
    Rml::FileInterface* file_interface = Rml::GetFileInterface();
    Rml::FileHandle file_handle = file_interface->Open(source);
    if (!file_handle)
    {
        return false;
    }

    file_interface->Seek(file_handle, 0, SEEK_END);
    size_t buffer_size = file_interface->Tell(file_handle);
    file_interface->Seek(file_handle, 0, SEEK_SET);

    RMLUI_ASSERTMSG(buffer_size > sizeof(TGAHeader), "Texture file size is smaller than TGAHeader, file must be corrupt or otherwise invalid");
    if(buffer_size <= sizeof(TGAHeader))
    {
        file_interface->Close(file_handle);
        return false;
    }

    char* buffer = new char[buffer_size];
    file_interface->Read(buffer, buffer_size, file_handle);
    file_interface->Close(file_handle);

    TGAHeader header;
    memcpy(&header, buffer, sizeof(TGAHeader));

    int color_mode = header.bitsPerPixel / 8;
    int image_size = header.width * header.height * 4; // We always make 32bit textures

    if (header.dataType != 2)
    {
        Rml::Log::Message(Rml::Log::LT_ERROR, "Only 24/32bit uncompressed TGAs are supported.");
        return false;
    }

    // Ensure we have at least 3 colors
    if (color_mode < 3)
    {
        Rml::Log::Message(Rml::Log::LT_ERROR, "Only 24 and 32bit textures are supported");
        return false;
    }

    const char* image_src = buffer + sizeof(TGAHeader);
    unsigned char* image_dest = new unsigned char[image_size];

    // Targa is BGR, swap to RGB and flip Y axis
    for (long y = 0; y < header.height; y++)
    {
        long read_index = y * header.width * color_mode;
        long write_index = ((header.imageDescriptor & 32) != 0) ? read_index : (header.height - y - 1) * header.width * color_mode;
        for (long x = 0; x < header.width; x++)
        {
            image_dest[write_index] = image_src[read_index+2];
            image_dest[write_index+1] = image_src[read_index+1];
            image_dest[write_index+2] = image_src[read_index];
            if (color_mode == 4)
                image_dest[write_index+3] = image_src[read_index+3];
            else
                image_dest[write_index+3] = 255;

            write_index += 4;
            read_index += color_mode;
        }
    }

    texture_dimensions.x = header.width;
    texture_dimensions.y = header.height;

    bool success = GenerateTexture(texture_handle, image_dest, texture_dimensions);

    delete [] image_dest;
    delete [] buffer;

    return success;
}

bool RocketRender::GenerateTexture(Rml::TextureHandle &texture_handle, const Rml::byte *source, const Rml::Vector2i &source_dimensions)
{
    GLuint texture_id = 0;
    glGenTextures(1, &texture_id);
    if (texture_id == 0)
    {
        fprintf(stdout,"Failed to generate textures\n");
        return false;
    }

    glBindTexture(GL_TEXTURE_2D, texture_id);

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, source_dimensions.x, source_dimensions.y, 0, GL_RGBA, GL_UNSIGNED_BYTE, source);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    texture_handle = (Rml::TextureHandle) texture_id;

    return true;
}

void RocketRender::ReleaseTexture(Rml::TextureHandle texture)
{
    glDeleteTextures(1, (GLuint*) &texture);
}

void RocketRender::SetTransform(const Rml::Matrix4f *transform)
{
    //TODO: the OpenGL state is not setup right for transforms yet.
    m_transformEnabled = (bool)transform;

    if (transform)
    {
        if (std::is_same<Rml::Matrix4f, Rml::ColumnMajorMatrix4f>::value)
            glLoadMatrixf(transform->data());
        else if (std::is_same<Rml::Matrix4f, Rml::RowMajorMatrix4f>::value)
            glLoadMatrixf(transform->Transpose().data());
    }
    else
        glLoadIdentity();
}