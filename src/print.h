#include <vector>

void print(const int &n) {
  std::cout << n << '\n';
}

void print(const double &n) {
  std::cout << n << '\n';
}

void print() {
  std::cout << '\n';
}

void print(const std::vector<double> &v) {
  if(v.empty()) {
    std::cout << "[]" << '\n';
    return;
  }
  std::cout << '[';
  for(int i = 0; i < v.size() - 1; ++i) {
    std::cout << v[i] << ", ";
  }
  std::cout << v.back() << ']' << '\n';
}

void print(const std::vector<int> &v) {
  if(v.empty()) {
    std::cout << "[]" << '\n';
    return;
  }
  std::cout << '[';
  for(int i = 0; i < v.size() - 1; ++i) {
    std::cout << v[i] << ", ";
  }
  std::cout << v.back() << ']' << '\n';
}

void print(const std::vector< std::vector<double> > &v) {
  if(v.empty()) {
    std::cout << "[[]]" << '\n';
    return;
  }
  std::cout << '[';
  for(int i = 0; i < v.size() - 1; ++i)
    print(v[i]);
  print(v.back());
  std::cout << ']' << '\n';
}

void print(const std::vector< std::vector<int> > &v) {
  if(v.empty()) {
    std::cout << "[[]]" << '\n';
    return;
  }
  std::cout << '[';
  for(int i = 0; i < v.size() - 1; ++i) 
    print(v[i]);
  print(v.back());
  std::cout << ']' << '\n';
}