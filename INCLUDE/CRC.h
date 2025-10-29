#ifndef CRC_H
#define CRC_H

#include <vector>
#include <cstdint>
#include <optional>

using namespace std;

class CRC {

    public:
        CRC();
        CRC(vector<int> denominator);

        vector<vector<int>> split12(vector<int> binaryData);
        vector<int> gather12(vector<vector<int>> splitBinaryData);

        uint16_t vec2bin(vector<int> binaryData);
        vector<int> bin2vec(uint16_t binaryData, int bitLength = 12);

        vector<uint16_t> encode1216(vector<uint16_t> splitBinaryData);
        bool verify(uint16_t encodedBinaryData);
        optional<uint16_t> decode1612(uint16_t encodedBinaryData);

        ~CRC();

    protected:
        vector<int> denominator = {1, 0, 0, 1, 1};

};

#endif