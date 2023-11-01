/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See GTSAM LICENSE for the license information

 * -------------------------------------------------------------------------- */

#pragma once

#include <string>
#include <utility>

namespace CORA {

typedef uint64_t Key;
class Symbol {
protected:
  unsigned char c_;
  uint64_t j_;

public:
  Symbol(unsigned char c, uint64_t j) : c_(c), j_(j) {}
  Symbol(const Symbol &other) : c_(other.c_), j_(other.j_) {}
  explicit Symbol(std::string s);
  explicit Symbol(Key key);
  uint64_t index() const { return j_; }
  unsigned char chr() const { return c_; }

  /// Conversion operators
  operator Key() const { return key(); }
  Key key() const;
  operator std::string() const;
  std::string string() const { return std::string(*this); }

  ///  Comparison operators
  bool operator==(const Symbol &other) const {
    return c_ == other.c_ && j_ == other.j_;
  }
  bool operator==(const Key &other) const { return key() == other; }
  bool operator!=(const Symbol &other) const { return !(*this == other); }
  bool operator!=(const Key &other) const { return !(*this == other); }
  bool operator<(const Symbol &other) const {
    return c_ < other.c_ || (c_ == other.c_ && j_ < other.j_);
  }
  bool operator<(const Key &other) const { return key() < other; }
};

using SymbolPair = std::pair<Symbol, Symbol>;
inline uint64_t symIndex(Key key) { return Symbol(key).index(); }
inline unsigned char symChar(Key key) { return Symbol(key).chr(); }
inline Key symbol(unsigned char c, uint64_t j) { return Symbol(c, j).key(); }

namespace shorthand {

inline Key A(std::uint64_t j) { return Symbol('a', j); }
inline Key B(std::uint64_t j) { return Symbol('b', j); }
inline Key C(std::uint64_t j) { return Symbol('c', j); }
inline Key D(std::uint64_t j) { return Symbol('d', j); }
inline Key E(std::uint64_t j) { return Symbol('e', j); }
inline Key F(std::uint64_t j) { return Symbol('f', j); }
inline Key G(std::uint64_t j) { return Symbol('g', j); }
inline Key H(std::uint64_t j) { return Symbol('h', j); }
inline Key I(std::uint64_t j) { return Symbol('i', j); }
inline Key J(std::uint64_t j) { return Symbol('j', j); }
inline Key K(std::uint64_t j) { return Symbol('k', j); }
inline Key L(std::uint64_t j) { return Symbol('l', j); }
inline Key M(std::uint64_t j) { return Symbol('m', j); }
inline Key N(std::uint64_t j) { return Symbol('n', j); }
inline Key O(std::uint64_t j) { return Symbol('o', j); }
inline Key P(std::uint64_t j) { return Symbol('p', j); }
inline Key Q(std::uint64_t j) { return Symbol('q', j); }
inline Key R(std::uint64_t j) { return Symbol('r', j); }
inline Key S(std::uint64_t j) { return Symbol('s', j); }
inline Key T(std::uint64_t j) { return Symbol('t', j); }
inline Key U(std::uint64_t j) { return Symbol('u', j); }
inline Key V(std::uint64_t j) { return Symbol('v', j); }
inline Key W(std::uint64_t j) { return Symbol('w', j); }
inline Key X(std::uint64_t j) { return Symbol('x', j); }
inline Key Y(std::uint64_t j) { return Symbol('y', j); }
inline Key Z(std::uint64_t j) { return Symbol('z', j); }

} // namespace shorthand
} // namespace CORA
