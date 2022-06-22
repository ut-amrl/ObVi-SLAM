//
// Created by amanda on 6/17/22.
//

#ifndef UT_VSLAM_BASIC_UTILS_H
#define UT_VSLAM_BASIC_UTILS_H

#include <boost/functional/hash.hpp>
#include <unordered_set>

namespace util {

template <typename EntryType>
using BoostHashSet = std::unordered_set<EntryType, boost::hash<EntryType>>;

template <typename KeyType, typename ValueType>
using BoostHashMap =
    std::unordered_map<KeyType, ValueType, boost::hash<KeyType>>;

struct EmptyStruct {};
}  // namespace util

#endif  // UT_VSLAM_BASIC_UTILS_H
