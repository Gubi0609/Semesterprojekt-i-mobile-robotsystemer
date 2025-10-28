# CRC Class Usage Guide
This guide shows how to use the CRC class.

# TODO
- [ ] Implement function to gather 12-bit binaries again
- [ ] Find other way to return error from decodign (`nullptr`?)

## Relevant files
  - `/INCLUDE/CRC.h` - Header file
  - `/SRC/CRC.cpp` - CRC implementation

## CRC API

### General Usage
Import libraries
```cpp
#include "../INCLUDE/CRC.h"

#include <iostream> // OPTIONAL. Used for printing to terminal
#include <cstdint> // Used for uint16_t (16 bit unsigned integer)
```
Create class instance
```cpp
CRC crc;
// OR include your wanted denominator (generator key) in init.
CRC crc = CRC(vector<int> {1, 0, 1, 0, 1}) // default = {1, 0, 0, 1, 1}
```

### Encoding Data Usage
Import your binary data (as a vector) and split in to 12 bit segments
```cpp
vector<int> binaryData = {1, 0, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0, 1, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0, 1, 0, 1};

vector<vector<int>> splitData = crc.split12(binaryData);
```
Convert 12 bit binary vectors to 16 bit (4 bit redundant) integers. Store in new vector
```cpp
vector<uint16_t> binData;

for(int i = 0; i < splitData.size(); i++) {
    uint16_t temp = crc.vec2bin(splitData[i]);

    binData.push_back(temp);
}
```
CRC encode the erro handling using CRC-4 (4 bit redundant) method
```cpp
vector<uint16_t> encodedData = crc.encode1216(binData); // encode1216 -> CRC-4
```

### Decoding Data Usage
To test the system, we append _one_ incorrect value to our data vector.
```cpp
encodedData.push_back(0x1A9C); // 0001 1010 1001 1100
```
We then decode the data. If a field is incorrect, 0000 0000 0000 0000 is returned for that field.
```cpp
vector<uint16_t> decodedData;

for(int i = 0; i < encodedData.size(); i++) {
    uint16_t temp = crc.decode1612(encodedData[i]);

    decodedData.push_back(temp);
}
```
The output is then
```bash
22
906
1813
0
```
Or in 12-bit  binary
```bash
0000 0001 0110
0011 1000 1010
0111 0001 0101
0000 0000 0000
```
