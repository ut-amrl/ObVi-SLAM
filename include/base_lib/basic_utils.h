//
// Created by amanda on 6/17/22.
//

#ifndef UT_VSLAM_BASIC_UTILS_H
#define UT_VSLAM_BASIC_UTILS_H

#include <boost/functional/hash.hpp>
#include <iomanip>
#include <string>
#include <unordered_set>

namespace util {

template <typename EntryType>
using BoostHashSet = std::unordered_set<EntryType, boost::hash<EntryType>>;

template <typename KeyType, typename ValueType>
using BoostHashMap =
    std::unordered_map<KeyType, ValueType, boost::hash<KeyType>>;

struct EmptyStruct {};

/**
 * Function to use to leading pad an integer number with zeros. This was
 * designed to work with positive integers that are certain to be less than or
 * equal to the given desired number of characters without padding, so take
 * caution and verify behavior if using in another scenario.
 *
 * @tparam NumType          Type of number to convert
 * @param number            Number to convert. Expected to be positive integer.
 * @param num_characters    Number of characters that the string should be
 *                          left-padded with zeros to be. Caller should make
 *                          sure this is greater than or equal to the number of
 *                          characters for the number without padding.
 *
 * @return Left-padded string for number.
 */
template <typename NumType>
std::string padIntNumberToConstantWidth(const NumType &number,
                                        const int &num_characters) {
  std::ostringstream padded_frame_string;
  padded_frame_string << std::setw(num_characters) << std::setfill('0')
                      << number;

  return padded_frame_string.str();
}
}  // namespace util

#endif  // UT_VSLAM_BASIC_UTILS_H
