#include "../INCLUDE/CRC.h"
#include <iostream>
#include <cmath>
#include <bitset>


using namespace std;

int main () {

    CRC crc = CRC();

    vector<uint16_t> rawData = {0b101100101101};
    
    vector<uint16_t> encodedData = crc.encode1216(rawData);

    uint16_t encodedDataRaw = encodedData[0];

    cout << bitset<16>(encodedDataRaw) << endl;

    int correctCheck = 0;

    for (int i = 0; i < pow(2, 16)-1; i++) {

        correctCheck += crc.verify(i);

    }

    cout << correctCheck << endl;
    
    int incorrect = pow(2,16) - correctCheck;

    double incorrectPercentile = incorrect/pow(2.0,16);
    double correctPercentile = correctCheck/pow(2.0,16);

    cout << "Percentile of incorrect binary: " << incorrectPercentile << endl;
    cout << "Percentile of correct binary: " << correctPercentile << endl;


    // Since bit-erros often comes in bursts instead of scrambling the whole data, we now operate with a 4 bit window instead:

    correctCheck = 0;
    correctCheck += crc.verify(encodedDataRaw);

    for (int shift = 0; shift <= 12; ++shift) {
        uint16_t mask = 0xF << shift;              // 4-bit window
        uint16_t nibble = (encodedDataRaw & mask) >> shift; // extract bits

        // Invert those 4 bits directly here
        uint16_t inverted = (~nibble) & 0xF;

        // Insert inverted bits back into original value
        uint16_t result = (encodedDataRaw & ~mask) | (inverted << shift);

        correctCheck += crc.verify(result);
    }

    cout << correctCheck << endl;
    incorrect = 14 - correctCheck;

    cout << incorrect << endl;

    incorrectPercentile = incorrect/14.0;
    correctPercentile = correctCheck/14.0;

    cout << "Percentile of incorrect binary: " << incorrectPercentile << endl;
    cout << "Percentile of correct binary: " << correctPercentile << endl;


    // We also try with a 2 bit window

    // Since bit-erros often comes in bursts instead of scrambling the whole data, we now operate with a 4 bit window instead:

    correctCheck = 0;
    correctCheck += crc.verify(encodedDataRaw);

    for (int shift = 0; shift <= 14; ++shift) {
        uint16_t mask = 0x3 << shift;              // 4-bit window
        uint16_t nibble = (encodedDataRaw & mask) >> shift; // extract bits

        // Invert those 4 bits directly here
        uint16_t inverted = (~nibble) & 0x3;

        // Insert inverted bits back into original value
        uint16_t result = (encodedDataRaw & ~mask) | (inverted << shift);

        correctCheck += crc.verify(result);
    }

    cout << correctCheck << endl;
    incorrect = 16 - correctCheck;

    cout << incorrect << endl;

    incorrectPercentile = incorrect/16.0;
    correctPercentile = correctCheck/16.0;

    cout << "Percentile of incorrect binary: " << incorrectPercentile << endl;
    cout << "Percentile of correct binary: " << correctPercentile << endl;


    // 4-bit window ALL POSSIBLE SCRAMBLES
    correctCheck = 0;
    correctCheck += crc.verify(encodedDataRaw);

    // Slide a 4-bit window from bit 0 to bit 12
    for (int shift = 0; shift <= 12; ++shift) {
        uint16_t mask = 0xF << shift;

        // Try all 16 possible 4-bit combinations
        for (uint16_t pattern = 0; pattern < 16; ++pattern) {
            // Replace those 4 bits with current pattern
            uint16_t result = (encodedDataRaw & ~mask) | (pattern << shift);

            correctCheck += crc.verify(result);
        }
    }

    cout << correctCheck << endl;
    incorrect = 209 - correctCheck;

    cout << incorrect << endl;

    incorrectPercentile = incorrect/209.0;
    correctPercentile = correctCheck/209.0;

    cout << "Percentile of incorrect binary: " << incorrectPercentile << endl;
    cout << "Percentile of correct binary: " << correctPercentile << endl;


    return 0;
}