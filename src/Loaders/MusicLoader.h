#pragma once

#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <set>
#include <stdio.h>
#include <boost/filesystem/path.hpp>

#include "../Util/Utils.h"

using namespace std;

typedef struct MAPHeader
{
    char szID[4];
    uint8_t bUnknown1;
    uint8_t bFirstSection;
    uint8_t bNumSections;
    uint8_t bRecordSize; // ???
    uint8_t Unknown2[3];
    uint8_t bNumRecords;
} MAPHeader;

typedef struct MAPSectionDefRecord
{
    uint8_t bUnknown;
    uint8_t bMagic;
    uint8_t bNextSection;
} MAPSectionDefRecord;

typedef struct MAPSectionDef
{
    uint8_t bIndex;
    uint8_t bNumRecords;
    uint8_t szID[2];
    struct MAPSectionDefRecord msdRecords[8];
} MAPSectionDef;

struct ASFBlockHeader
{
    char szBlockID[4];
    uint32_t dwSize;
};

struct ASFChunkHeader
{
    uint32_t dwOutSize;
    uint16_t lCurSampleLeft;
    uint16_t lPrevSampleLeft;
    uint16_t lCurSampleRight;
    uint16_t lPrevSampleRight;
};

struct OldBNKHeader
{
    char szID[4];
    uint16_t wVersion; // 0x0002 for "old" header format; 0x0004 for "new" header format (see below)
    uint16_t wNumberOfSounds; // Number of sounds inside the BNK
    uint32_t dwFirstSoundStart; // Start of sound data
    uint32_t dwSoundsArray[];  // Position of the PT headers for each sound.  wNumberOfSounds entries long.  Read rest of header first and then handle this.
};

struct NewBNKHeader
{
    char szID[4];
    uint16_t wVersion; // Should be 0x0004
    uint16_t wNumberOfSounds; // Number of sounds inside the BNK
    uint32_t dwFirstSoundStart; // Start of sound data
    uint32_t dwSoundSize; // = total filesize - dwFirstSoundStart
    uint32_t dwUnknown; // seems to contain small number <20 or -1
    uint32_t dwSoundsArray[];  // Position of the PT headers for each sound.  wNumberOfSounds entries long.  Read rest of header first and then handle this.
};

struct PTHeader
{
    uint32_t dwSampleRate;
    uint32_t dwChannels;
    uint32_t dwCompression;
    uint32_t dwNumSamples;
    uint32_t dwDataStart;
    uint32_t dwLoopOffset;
    uint32_t dwLoopLength;
    uint32_t dwBytesPerSample;
    uint32_t bSplit;
    uint32_t bSplitCompression;

    PTHeader()
    {
        dwSampleRate = 22050;
        dwChannels = 2;
        dwCompression = 0;
        dwBytesPerSample = 2;
    }
};

class AudioLoader
{
protected:
    uint32_t EATable[20] = {0x00000000, 0x000000F0, 0x000001CC, 0x00000188, 0x00000000, 0x00000000, 0xFFFFFF30, 0xFFFFFF24, 0x00000000, 0x00000001,
                            0x00000003, 0x00000004, 0x00000007, 0x00000008, 0x0000000A, 0x0000000B, 0x00000000, 0xFFFFFFFF, 0xFFFFFFFD, 0xFFFFFFFC};

    uint32_t ReadBytes(FILE *file, uint8_t count);

    bool ReadSCHl(FILE *mus_file, uint32_t sch1Offset, FILE *pcm_file);

    void DecompressEAADPCM(ASFChunkHeader *asfChunkHeader, long nSamples, FILE *mus_file, FILE *pcm_file);

    void ParsePTHeader(FILE *file,
                       uint32_t *dwSampleRate,
                       uint32_t *dwChannels,
                       uint32_t *dwCompression,
                       uint32_t *dwNumSamples,
                       uint32_t *dwDataStart,
                       uint32_t *dwLoopOffset,
                       uint32_t *dwLoopLength,
                       uint32_t *dwBytesPerSample,
                       uint32_t *bSplit,
                       uint32_t *bSplitCompression);
};

class AudioBuffer
{
public:
    void SetHeader(PTHeader inheader)
    {
        header = inheader;
    };
    void SetBuffer(char *inbuffer)
    {
        buffer = inbuffer;
    };

    PTHeader* GetHeaderPtr()
    {
        return &header;
    };
    char* GetBufPtr()
    {
        return buffer;
    };

private:
    PTHeader header;
    char* buffer;
};

class MusicLoader : public AudioLoader
{
public:
    explicit MusicLoader(const std::string &song_base_path);
    void ParseMAP(const std::string &map_path, const std::string &mus_path);
};

class BnkLoader : public AudioLoader
{
public:
    std::vector<AudioBuffer> LoadBnk(const std::string &bnk_base_path);

private:
    void LoadNewBnk(FILE *bnkFile, NewBNKHeader *bnkHeader, AudioBuffer *buffer, std::vector<AudioBuffer> &vBufferArray);
};