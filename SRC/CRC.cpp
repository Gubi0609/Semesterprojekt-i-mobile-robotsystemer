#include "../INCLUDE/CRC.h"

#include <bits/stdc++.h>
#include <iostream>

/*
TODO
- [ ] Function to gather 12 bit binary again in long vector
- [ ] Other return type from decode1612 (nullptr?)
*/

CRC::CRC() {}

CRC::CRC(vector<int> denominator) {
    this->denominator = denominator;
}

vector<vector<int>> CRC::split8(vector<int> binaryData) {
    vector<vector<int>> splitBinary;

    for(int i = binaryData.size() - 1; i >= 0;) {
        vector<int> group;

        for(int j = 0; j < 8 && i>=0; j++, i--) {
            
            group.push_back(binaryData[i]);
        }

        reverse(group.begin(), group.end());
        splitBinary.push_back(group);
    }

    reverse(splitBinary.begin(), splitBinary.end());

    if(!splitBinary.empty() && splitBinary[0].size() < 8) {
        int spacesMissing = 8 - splitBinary[0].size();

        for (int i = 0; i < spacesMissing; i++) {
            splitBinary[0].insert(splitBinary[0].begin(), 0);
        }
    }

    return splitBinary;

}

vector<vector<int>> CRC::split12(vector<int> binaryData) {
    vector<vector<int>> splitBinary;

    for(int i = binaryData.size() - 1; i >= 0;) {
        vector<int> group;

        for(int j = 0; j < 12 && i>=0; j++, i--) {
            
            group.push_back(binaryData[i]);
        }

        reverse(group.begin(), group.end());
        splitBinary.push_back(group);
    }

    reverse(splitBinary.begin(), splitBinary.end());

    if(!splitBinary.empty() && splitBinary[0].size() < 12) {
        int spacesMissing = 12 - splitBinary[0].size();

        for (int i = 0; i < spacesMissing; i++) {
            splitBinary[0].insert(splitBinary[0].begin(), 0);
        }
    }

    return splitBinary;

}

uint16_t CRC::vec2bin(vector<int> binaryData) {
    
   
    uint16_t result = 0;
    
    for (int bit : binaryData) {
            if (bit != 0 && bit != 1) {
                throw std::invalid_argument("Bits must be 0 or 1");
            }
            result = (result << 1) | bit;
        }

    return result;
   
}

vector<int> CRC::bin2vec(uint16_t binaryData) {
 
    vector<int> bits;

    for (int j = 15; j >= 0; --j) {
        bits.push_back((binaryData >> j) & 1);
    }
    
    return bits;
    
}

vector<uint16_t> CRC::encode1216(vector<uint16_t> splitBinaryData) {
    
    vector<uint16_t> encodedData;

    // Generator polynomial for class denominator
    const uint16_t generator = vec2bin(denominator);

    for(int i = 0; i < splitBinaryData.size(); i++) {
        // We only use 12 bits of data
        splitBinaryData[i] &= 0x0FFF;

        // Append 4 zeros (shift left by 4 bits)
        uint16_t shifted = splitBinaryData[i] << 4;

        // Perform modulo-2 division
        for (int i = 15; i >= 4; --i) { // from MSB to bit 4
            if (shifted & (1 << i)) {
                shifted ^= generator << (i - 4);
            }
        }

        // Remainder is in the lowest 4 bits
        uint16_t crc = shifted & 0xF;

        // Encoded 16-bit word: data (12 bits) + crc (4 bits)
        encodedData.push_back((splitBinaryData[i] << 4) | crc);

    }

    return encodedData;

}

bool CRC::verify(uint16_t received) {
    const uint16_t generator = vec2bin(denominator);
    uint16_t temp = received;

    // Perform modulo-2 division
    for (int i = 15; i >= 4; --i) {
        if (temp & (1 << i)) {
            temp ^= generator << (i - 4);
        }
    }

    // If remainder == 0, CRC is valid
    return (temp & 0xF) == 0;
}

uint16_t CRC::decode1612(uint16_t encodedBinaryData) {
    
    
    // Check CRC first
    bool isValid = verify(encodedBinaryData);

    if (isValid) {
        // Extract 12-bit data regardless
        return (encodedBinaryData >> 4) & 0x0FFF;
    } else {
        return 0x0000;
    }

}

CRC::~CRC() {}