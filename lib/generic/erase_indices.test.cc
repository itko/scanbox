/// \file
/// Maintainer: Felice Serena
///
///

#include "erase_indices.h"

#include <boost/test/unit_test.hpp>

using namespace MouseTrack;

BOOST_AUTO_TEST_CASE(erase_empty_vec) {
  std::vector<int> vec;
  std::vector<int> remove;
  remove.push_back(0);

  erase_indices(vec, remove);

  BOOST_CHECK_EQUAL(vec.size(), 0);
}

BOOST_AUTO_TEST_CASE(erase_empty_remove) {
  std::vector<int> vec;
  std::vector<int> remove;
  vec.push_back(0);

  erase_indices(vec, remove);

  BOOST_CHECK_EQUAL(vec.size(), 1);
}

BOOST_AUTO_TEST_CASE(erase_remove_first) {
  std::vector<int> vec;
  std::vector<int> remove;
  vec.push_back(13);
  vec.push_back(15);
  vec.push_back(17);
  vec.push_back(19);

  remove.push_back(0);

  erase_indices(vec, remove);

  BOOST_CHECK_EQUAL(vec.size(), 3);

  BOOST_CHECK_EQUAL(vec[0], 15);
  BOOST_CHECK_EQUAL(vec[1], 17);
  BOOST_CHECK_EQUAL(vec[2], 19);
}

BOOST_AUTO_TEST_CASE(erase_remove_middle) {
  std::vector<int> vec;
  std::vector<int> remove;
  vec.push_back(13);
  vec.push_back(15);
  vec.push_back(17);
  vec.push_back(19);

  remove.push_back(2);

  erase_indices(vec, remove);

  BOOST_CHECK_EQUAL(vec.size(), 3);

  BOOST_CHECK_EQUAL(vec[0], 13);
  BOOST_CHECK_EQUAL(vec[1], 15);
  BOOST_CHECK_EQUAL(vec[2], 19);
}

BOOST_AUTO_TEST_CASE(erase_remove_end) {
  std::vector<int> vec;
  std::vector<int> remove;
  vec.push_back(13);
  vec.push_back(15);
  vec.push_back(17);
  vec.push_back(19);

  remove.push_back(3);

  erase_indices(vec, remove);

  BOOST_CHECK_EQUAL(vec.size(), 3);

  BOOST_CHECK_EQUAL(vec[0], 13);
  BOOST_CHECK_EQUAL(vec[1], 15);
  BOOST_CHECK_EQUAL(vec[2], 17);
}

BOOST_AUTO_TEST_CASE(erase_remove_two_start_middle) {
  std::vector<int> vec;
  std::vector<int> remove;
  vec.push_back(13);
  vec.push_back(15);
  vec.push_back(17);
  vec.push_back(19);
  vec.push_back(23);

  remove.push_back(0);
  remove.push_back(3);

  erase_indices(vec, remove);

  BOOST_CHECK_EQUAL(vec.size(), 3);

  BOOST_CHECK_EQUAL(vec[0], 15);
  BOOST_CHECK_EQUAL(vec[1], 17);
  BOOST_CHECK_EQUAL(vec[2], 23);
}

BOOST_AUTO_TEST_CASE(erase_remove_two_start_end) {
  std::vector<int> vec;
  std::vector<int> remove;
  vec.push_back(13);
  vec.push_back(15);
  vec.push_back(17);
  vec.push_back(19);
  vec.push_back(23);

  remove.push_back(0);
  remove.push_back(4);

  erase_indices(vec, remove);

  BOOST_CHECK_EQUAL(vec.size(), 3);

  BOOST_CHECK_EQUAL(vec[0], 15);
  BOOST_CHECK_EQUAL(vec[1], 17);
  BOOST_CHECK_EQUAL(vec[2], 19);
}

BOOST_AUTO_TEST_CASE(erase_remove_two_middle_middle) {
  std::vector<int> vec;
  std::vector<int> remove;
  vec.push_back(13);
  vec.push_back(15);
  vec.push_back(17);
  vec.push_back(19);
  vec.push_back(23);

  remove.push_back(1);
  remove.push_back(3);

  erase_indices(vec, remove);

  BOOST_CHECK_EQUAL(vec.size(), 3);

  BOOST_CHECK_EQUAL(vec[0], 13);
  BOOST_CHECK_EQUAL(vec[1], 17);
  BOOST_CHECK_EQUAL(vec[2], 23);
}

BOOST_AUTO_TEST_CASE(erase_remove_two_middle_end) {
  std::vector<int> vec;
  std::vector<int> remove;
  vec.push_back(13);
  vec.push_back(15);
  vec.push_back(17);
  vec.push_back(19);
  vec.push_back(23);

  remove.push_back(1);
  remove.push_back(4);

  erase_indices(vec, remove);

  BOOST_CHECK_EQUAL(vec.size(), 3);

  BOOST_CHECK_EQUAL(vec[0], 13);
  BOOST_CHECK_EQUAL(vec[1], 17);
  BOOST_CHECK_EQUAL(vec[2], 19);
}

BOOST_AUTO_TEST_CASE(erase_remove_two_sequential_start) {
  std::vector<int> vec;
  std::vector<int> remove;
  vec.push_back(13);
  vec.push_back(15);
  vec.push_back(17);
  vec.push_back(19);
  vec.push_back(23);

  remove.push_back(0);
  remove.push_back(1);

  erase_indices(vec, remove);

  BOOST_CHECK_EQUAL(vec.size(), 3);

  BOOST_CHECK_EQUAL(vec[0], 17);
  BOOST_CHECK_EQUAL(vec[1], 19);
  BOOST_CHECK_EQUAL(vec[2], 23);
}

BOOST_AUTO_TEST_CASE(erase_remove_two_sequential_middle) {
  std::vector<int> vec;
  std::vector<int> remove;
  vec.push_back(13);
  vec.push_back(15);
  vec.push_back(17);
  vec.push_back(19);
  vec.push_back(23);

  remove.push_back(2);
  remove.push_back(3);

  erase_indices(vec, remove);

  BOOST_CHECK_EQUAL(vec.size(), 3);

  BOOST_CHECK_EQUAL(vec[0], 13);
  BOOST_CHECK_EQUAL(vec[1], 15);
  BOOST_CHECK_EQUAL(vec[2], 23);
}

BOOST_AUTO_TEST_CASE(erase_remove_two_sequential_end) {
  std::vector<int> vec;
  std::vector<int> remove;
  vec.push_back(13);
  vec.push_back(15);
  vec.push_back(17);
  vec.push_back(19);
  vec.push_back(23);

  remove.push_back(3);
  remove.push_back(4);

  erase_indices(vec, remove);

  BOOST_CHECK_EQUAL(vec.size(), 3);

  BOOST_CHECK_EQUAL(vec[0], 13);
  BOOST_CHECK_EQUAL(vec[1], 15);
  BOOST_CHECK_EQUAL(vec[2], 17);
}
