/*
 * basic_tools.h
 *
 *  Created on: 22 Jan 2018
 *      Author: footstool
 */

#ifndef CVSSP_TOOLS_INCLUDE_CVSSP_TOOLS_BASIC_TOOLS_H_
#define CVSSP_TOOLS_INCLUDE_CVSSP_TOOLS_BASIC_TOOLS_H_
#include <iostream>

namespace cvssp_tools{
template<typename T>
static bool AreEqual(T f1, T f2) {
  return (std::fabs(f1 - f2) <= std::numeric_limits<T>::epsilon() * std::fmax(fabs(f1), fabs(f2)));
}

bool fileExists(const std::string& name)
{
  std::ifstream f(name.c_str());
  return f.good();
}
int getFileNumLines(const std::string& filename)
{
  std::ifstream file;
  file.open(filename);
  std::string line;
  int number_of_lines = 0;
  for (std::string line; std::getline(file, line); number_of_lines++)
    ;

  file.close();
  return number_of_lines;
}
}
#endif /* CVSSP_TOOLS_INCLUDE_CVSSP_TOOLS_BASIC_TOOLS_H_ */
