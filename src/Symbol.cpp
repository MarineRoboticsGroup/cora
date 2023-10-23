/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Symbol.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-10-23
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <CORA/Symbol.h>

namespace CORA {

static const size_t keyBits = sizeof(Key) * 8;
static const size_t chrBits = sizeof(unsigned char) * 8;
static const size_t indexBits = keyBits - chrBits;
static const Key chrMask =
    Key(UCHAR_MAX)
    << indexBits; // For some reason, std::numeric_limits<unsigned char>::max()
                  // fails
static const Key indexMask = ~chrMask;

Symbol::Symbol(std::string s) : c_(s[0]), j_(std::stoull(s.substr(1))) {}
Symbol::Symbol(Key key) : c_(key >> indexBits), j_(key & indexMask) {}

Key Symbol::key() const { return (Key(c_) << indexBits) | j_; }

} // namespace CORA
