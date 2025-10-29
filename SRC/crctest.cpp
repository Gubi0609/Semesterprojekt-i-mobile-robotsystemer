#include "../INCLUDE/CRC.h"

#include <iostream>
#include <cstdint>


int main() {

    CRC crc;

    cout << "--- Split data --- " << endl; 

    vector<int> binaryData = {1, 0, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0, 1, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0, 1, 0, 1};
    // vector<int> binaryData = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 ,13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28};

    vector<vector<int>> splitData = crc.split12(binaryData);

    for (int i = 0; i < splitData.size(); i++) {
        for (int j = 0; j < splitData[i].size(); j++) {
            cout << splitData[i][j] << " ";
        }
        cout << endl;
    }

    cout << "--- Convert to bin ---" << endl;

    vector<uint16_t> binData;

    for(int i = 0; i < splitData.size(); i++) {
        uint16_t temp = crc.vec2bin(splitData[i]);

        binData.push_back(temp);
    }

    for(int i = 0; i < binData.size(); i++) {
        cout << binData[i] << endl;
    }

    cout << "--- Encoding data ---" << endl;

    vector<uint16_t> encodedData = crc.encode1216(binData);

    for(int i = 0; i < encodedData.size(); i++) {
        cout << encodedData[i] << endl;
    }

    cout << "--- Adding incorrect value ---" << endl;

    encodedData.push_back(0x1A9C);

    cout << "--- Decoding data ---" << endl;

    vector<uint16_t> decodedData;
    vector<int> errorBlocks;
    
    for(int i = 0; i < encodedData.size(); i++) {
        auto temp = crc.decode1612(encodedData[i]);

        if(temp != nullopt) {
            decodedData.push_back(*temp);
        } else {
            errorBlocks.push_back(i);
        }

    }

    if(!errorBlocks.empty()) {
        // Do some error handling
        cout << "ERROR DETECTED IN " << errorBlocks.size() << " BLOCKS!" << endl;
    }

    for(int i = 0; i < decodedData.size(); i++) {
        cout << decodedData[i] << endl;
    }

    vector<vector<int>> decodedSplitData;

    for(int i = 0; i < decodedData.size(); i++) {
        decodedSplitData.push_back(crc.bin2vec(decodedData[i]));
    }

    for (int i = 0; i < decodedSplitData.size(); i++) {
        for (int j = 0; j < decodedSplitData[i].size(); j++) {
            cout << decodedSplitData[i][j] << " ";
        }
        cout << endl;
    }

    vector<int> gatheredDecodedData;

    for(int i = 0; i < decodedSplitData.size(); i++) {
        for(int j = 0; j < decodedSplitData[i].size(); j++) {
            gatheredDecodedData.push_back(decodedSplitData[i][j]);
        }
    }

    for(int i = 0; i < gatheredDecodedData.size(); i++) {
        cout << gatheredDecodedData[i] << " ";
    }
    cout << endl;
    for(int i = 0; i < binaryData.size(); i++) {
        cout << binaryData[i] << " ";
    }
    cout << endl;
    
}