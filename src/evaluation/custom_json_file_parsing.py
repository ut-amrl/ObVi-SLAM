

class CustomJsonConstants:
    # C++ Optional
    hasValueKey = "has_v"
    valueKey = "v"
    indexKey = "i"

def readUtVSLAMVector(jsonForVector):
    extractedData = []
    for indivEntryJson in jsonForVector:
        extractedData.append(indivEntryJson[CustomJsonConstants.valueKey])
    return extractedData