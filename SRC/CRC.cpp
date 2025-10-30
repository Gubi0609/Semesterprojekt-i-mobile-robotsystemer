#include "../INCLUDE/CRC.h"

#include <bits/stdc++.h>
#include <iostream>

CRC::CRC() {}

CRC::CRC(vector<int> generatorKey) {
    /*
    Initializes a CRC object
    :param generatorKey: The binary key (in vector form) to be used to calculate CRC
    */

    this->generatorKey = generatorKey;
}

vector<vector<int>> CRC::split(vector<int> binaryData, int desiredLength) {
    /*
    Splits a long vector in to elements of desiredLength, starting from LSB (the back).
    :param binaryData: Datavector to be split
    :param desiredLength: Desired length of the processed binary vectors. Default = 12
    :return: Binary data split in to lengths of desiredLength
    */

    vector<vector<int>> splitBinary;

    for(int i = binaryData.size() - 1; i >= 0;) {
        vector<int> group;

        for(int j = 0; j < desiredLength && i>=0; j++, i--) {
            
            group.push_back(binaryData[i]);
        }

        reverse(group.begin(), group.end());
        splitBinary.push_back(group);
    }

    reverse(splitBinary.begin(), splitBinary.end());

    if(!splitBinary.empty() && splitBinary[0].size() < desiredLength) {
        int spacesMissing = desiredLength - splitBinary[0].size();

        for (int i = 0; i < spacesMissing; i++) {
            splitBinary[0].insert(splitBinary[0].begin(), 0);
        }
    }

    return splitBinary;

}

vector<int> CRC::gather(vector<vector<int>> splitBinaryData) {
    /*
    Gathers elements from seperate vectors to one long vector.
    :param splitBinaryData: Split data to be gathered
    :return: splitBinaryData gathered in to one long vector
    */

    vector<int> gatheredBinaryData;

    for(int i = 0; i < splitBinaryData.size(); i++) {
        for(int j = 0; j < splitBinaryData[i].size(); j++) {
            gatheredBinaryData.push_back(splitBinaryData[i][j]);
        }
    }

    return gatheredBinaryData;
}

uint16_t CRC::vec2bin(vector<int> binaryData) {
    /*
    Converts a binary vector in to a 16 bit binary number.
    :param binaryData: Vector form of data to be converted to binary
    :return: 16 bit binary data from binaryData vector
    */
   
    uint16_t result = 0;
    
    if(binaryData.size()>16) {
        throw std::invalid_argument("Maximum length of vector is 16");
    }

    for (int bit : binaryData) {
            if (bit != 0 && bit != 1) {
                throw std::invalid_argument("Bits must be 0 or 1");
            }
            result = (result << 1) | bit;
        }

    return result;
   
}

vector<int> CRC::bin2vec(uint16_t binaryData, int bitLength) {
    /*
    Converts a 16 bit binary number to a binary vector.
    :param binaryData: 16 bit binary number to be converted to vector form
    :param bitLength: Number of significant bits in binaryData. Default = 12
    :return: Vector form of binaryData
    */
 
    if (bitLength>16) {
        throw std::invalid_argument("Maximum length of binary is 16");
    }

    vector<int> bits;
    bits.reserve(bitLength);

    for (int j = bitLength - 1; j >= 0; --j) {
        bits.push_back((binaryData >> j) & 1);
    }

    return bits;
    
}

vector<uint16_t> CRC::encode1216(vector<uint16_t> splitBinaryData) {
    /*
    CRC-4 encodes a vector of 12 bit binary numbers to 16 bit binary numbers using CRC generatorKey
    :param splitBinaryData: 12 bit binary datas vector to be CRC-4 encoded
    :return: CRC-14 encoded 16 bit binary data
    */
    
    vector<uint16_t> encodedData;

    // Generator polynomial for class denominator
    const uint16_t generator = vec2bin(generatorKey);

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
    /*
    Verifies if CRC-4 encoded data is uncorrupted.
    :param received: CRC-4 encoded 16 bit binary to be verified
    :return: bool True/False. True if uncorrupted, False if corrupted
    */

    const uint16_t generator = vec2bin(generatorKey);
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

optional<uint16_t> CRC::decode1612(uint16_t encodedBinaryData) {
    /*
    Verifies and decodes a 16 bit CRC-4 binary to a 12 bit data binary
    :param encodedBinaryData: 16 bit CRC-4 data to be decoded
    :return: nullopt if data is corrupted. 12 bit decoded data binary if uncorrupted
    */
    bool isValid = verify(encodedBinaryData);

    if (!isValid)
        return nullopt; // indicates error

    uint16_t extractedData = (encodedBinaryData >> 4) & 0x0FFF;
    return extractedData; // return by value
}


CRC::~CRC() {}