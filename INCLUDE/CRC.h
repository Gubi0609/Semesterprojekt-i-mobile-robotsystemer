#ifndef CRC_H
#define CRC_H

#include <vector>
#include <cstdint>

using namespace std;

class CRC {

    public:
        CRC();
        CRC(vector<int> denominator);

        vector<vector<int>> split8(vector<int> binaryData);
        vector<vector<int>> split12(vector<int> binaryData);

        uint16_t vec2bin(vector<int> binaryData);
        vector<int> bin2vec(uint16_t binaryData);

        vector<uint16_t> encode1216(vector<uint16_t> splitBinaryData);
        bool verify(uint16_t encodedBinaryData);
        uint16_t decode1612(uint16_t encodedBinaryData);

        ~CRC();

    protected:
        vector<int> denominator = {1, 0, 0, 1, 1};

};

#endif