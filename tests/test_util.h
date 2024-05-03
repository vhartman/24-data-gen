#pragma once

#include <iostream>
#include <string>

void report_test_result(const std::string& text, const bool success){
  if (success){
     std::cout << "\033[1;32m" << text << "\033[0m" << std::endl;
  }
  else{
     std::cout << "\033[1;31m" << text << "\033[0m" << std::endl;
  }
}