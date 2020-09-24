#ifndef KISAKSTRIKE_ROCKETFILESYSTEM_H
#define KISAKSTRIKE_ROCKETFILESYSTEM_H

#include <RmlUi/Core/FileInterface.h>

class RocketFileSystem : public Rml::FileInterface
{
    /** singleton support **/
public:
    static RocketFileSystem m_Instance;
    RocketFileSystem();

    /// Opens a file.
    /// @param file The file handle to write to.
    /// @return A valid file handle, or nullptr on failure
    virtual Rml::FileHandle Open(const Rml::String& path) override;
    /// Closes a previously opened file.
    /// @param file The file handle previously opened through Open().
    virtual void Close(Rml::FileHandle file) override;

    /// Reads data from a previously opened file.
    /// @param buffer The buffer to be read into.
    /// @param size The number of bytes to read into the buffer.
    /// @param file The handle of the file.
    /// @return The total number of bytes read into the buffer.
    virtual size_t Read(void* buffer, size_t size, Rml::FileHandle file) override;
    /// Seeks to a point in a previously opened file.
    /// @param file The handle of the file to seek.
    /// @param offset The number of bytes to seek.
    /// @param origin One of either SEEK_SET (seek from the beginning of the file), SEEK_END (seek from the end of the file) or SEEK_CUR (seek from the current file position).
    /// @return True if the operation completed successfully, false otherwise.
    virtual bool Seek(Rml::FileHandle file, long offset, int origin) override;
    /// Returns the current position of the file pointer.
    /// @param file The handle of the file to be queried.
    /// @return The number of bytes from the origin of the file.
    virtual size_t Tell(Rml::FileHandle file) override;

    /// Returns the length of the file.
    /// The default implementation uses Seek & Tell.
    /// @param file The handle of the file to be queried.
    /// @return The length of the file in bytes.
    virtual size_t Length(Rml::FileHandle file) override;
};

#endif //KISAKSTRIKE_ROCKETFILESYSTEM_H