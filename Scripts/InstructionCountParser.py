#!/usr/bin/python3
import base64
from dataclasses import dataclass, field
from enum import Flag
import json
import struct
import sys
import subprocess
import os
import logging
logger = logging.getLogger()
logger.setLevel(logging.ERROR)

@dataclass
class TestData:
    name: str
    optimal: int
    expectedinstructioncount: int
    code: bytes
    def __init__(self, Name, Optimal, ExpectedInstructionCount, Code):
        self.name = Name
        self.expectedinstructioncount = ExpectedInstructionCount
        self.optimal = Optimal
        self.code = Code

    @property
    def Name(self):
        return self.name

    @property
    def Optimal(self):
        return self.optimal

    @property
    def ExpectedInstructionCount(self):
        return self.expectedinstructioncount

    @property
    def Code(self):
        return self.code

TestDataMap = {}
class HostFeatures(Flag) :
    FEATURE_ANY    = 0
    FEATURE_SVE128 = (1 << 0)
    FEATURE_SVE256 = (1 << 1)
    FEATURE_CLZERO = (1 << 2)
    FEATURE_RNG    = (1 << 3)
    FEATURE_FCMA   = (1 << 4)

HostFeaturesLookup = {
    "SVE128"  : HostFeatures.FEATURE_SVE128,
    "SVE256"  : HostFeatures.FEATURE_SVE256,
    "CLZERO"  : HostFeatures.FEATURE_CLZERO,
    "RNG"     : HostFeatures.FEATURE_RNG,
    "FCMA"    : HostFeatures.FEATURE_FCMA,
}

def GetHostFeatures(data):
    HostFeaturesData = HostFeatures.FEATURE_ANY
    if not (type(data) is list):
        sys.exit("Features value must be list of features")

    for data_key in data:
        data_key = data_key.upper()
        if not (data_key in HostFeaturesLookup):
            sys.exit("Invalid host feature")

        HostFeaturesData |= HostFeaturesLookup[data_key]
    return HostFeaturesData

def parse_json_data(json_filename, json_data, output_binary_path):
    Bitness = 64
    EnabledHostFeatures = HostFeatures.FEATURE_ANY
    DisabledHostFeatures = HostFeatures.FEATURE_ANY

    if "Features" in json_data:
        items = json_data["Features"]
        if ("Bitness" in items):
            Bitness = int(items["Bitness"])

        if ("EnabledHostFeatures" in items):
            EnabledHostFeatures = GetHostFeatures(items["EnabledHostFeatures"])

        if ("DisabledHostFeatures" in items):
            DisabledHostFeatures = GetHostFeatures(items["DisabledHostFeatures"])

    for key, items in json_data["Instructions"].items():
        ExpectedInstructionCount = 0
        Optimal = 0
        if ("ExpectedInstructionCount" in items):
            ExpectedInstructionCount = int(items["ExpectedInstructionCount"])

        if ("Optimal" in items):
                if items["Optimal"].upper() == "YES":
                    Optimal = 1

        if ("Skip" in items):
                if items["Skip"].upper() == "YES":
                    continue

        TestName = base64.b64encode("{}.{}".format(json_filename, key).encode("ascii")).decode("ascii")
        tmp_asm = "/tmp/{}.asm".format(TestName)
        tmp_asm_out = "/tmp/{}.asm.o".format(TestName)
        logging.info("'{}' -> '{}' -> '{}'".format(key, tmp_asm, tmp_asm_out))

        with open(tmp_asm, "w") as tmp_asm_file:
            tmp_asm_file.write("BITS {};\n".format(Bitness))
            tmp_asm_file.write("{}\n".format(key))

        Process = subprocess.Popen(["nasm", tmp_asm, "-o", tmp_asm_out])
        Process.wait()
        ResultCode = Process.returncode

        if ResultCode != 0:
            os.remove(tmp_asm)
            logging.error("Nasm failed to execute")
            logging.error("Couldn't compile: '{}'".format(key))
            return ResultCode

        if not os.path.exists(tmp_asm_out):
            logging.error("Nasm didn't emit code?")
            os.remove(tmp_asm)
            return 1

        logging.info("Generated asm file")

        with open(tmp_asm_out, "rb") as tmp_asm_out_file:
            binary_hex = tmp_asm_out_file.read()

        TestDataMap[TestName] = TestData(key, Optimal, ExpectedInstructionCount, binary_hex)

        os.remove(tmp_asm)
        os.remove(tmp_asm_out)

        # Output the test data as follows
        # struct TestInfo;
        # struct DataHeader {
        #   uint64_t Bitness;
        #   uint64_t NumTests;
        #   uint64_t EnabledHostFeatures;
        #   uint64_t DisabledHostFeatures;
        #   TestInfo Tests[NumTests];
        # };
        # struct TestInfo {
        #   char InstName[128];
        #   uint64_t Optimal;
        #   int64_t ExpectedInstructionCount;
        #   uint64_t CodeSize;
        #   uint32_t Cookie;
        #   uint8_t Code[CodeSize];
        # };

    MemData = bytes()
    # Add the header
    MemData += struct.pack('Q', Bitness)
    MemData += struct.pack('Q', len(TestDataMap))
    MemData += struct.pack('Q', EnabledHostFeatures.value)
    MemData += struct.pack('Q', DisabledHostFeatures.value)

    # Add each test
    for key, item in TestDataMap.items():
        MemData += struct.pack('128s', item.Name.encode("ascii"))
        MemData += struct.pack('Q', item.Optimal)
        MemData += struct.pack('q', item.ExpectedInstructionCount)
        MemData += struct.pack('Q', len(item.Code))
        MemData += struct.pack('I', 0x41424344)
        MemData += item.Code

    logging.info("Code goign to {}".format(output_binary_path))
    with open(output_binary_path, "wb") as output_binary_file:
        output_binary_file.write(MemData)

    return 0

def main():
    if sys.version_info[0] < 3:
        logging.critical ("Python 3 or a more recent version is required.")

    if (len(sys.argv) < 3):
        logging.critical ("usage: %s <PerformanceTests.json> <output_folder>" % (sys.argv[0]))

    json_path = sys.argv[1]
    output_binary_path = sys.argv[2]

    try:
        with open(json_path) as json_file:
            json_text = json_file.read()
    except IOError:
        logging.error("IOError!")
        return 1

    try:
        json_data = json.loads(json_text)
        if not isinstance(json_data, dict):
            raise TypeError('JSON data must be a dict')

        return parse_json_data(os.path.basename(json_path), json_data, output_binary_path)

    except ValueError as ve:
        logging.error(f'JSON error: {ve}')

        return 1

    return 0

if __name__ == "__main__":
# execute only if run as a script
    sys.exit(main())
