#include "PolynomialMap.h"
#include <fstream>
#include <algorithm>
#include <cmath>
#include <iostream>

using namespace std;

// 拷贝构造函数
PolynomialMap::PolynomialMap(const PolynomialMap& other) 
    : m_Polynomial(other.m_Polynomial) {}

// 从数组构造多项式
PolynomialMap::PolynomialMap(const double* cof, const int* deg, int n) {
    for (int i = 0; i < n; ++i) {
        if (cof[i] != 0.0) {
            m_Polynomial[deg[i]] += cof[i];
            if (m_Polynomial[deg[i]] == 0.0) {
                m_Polynomial.erase(deg[i]);
            }
        }
    }
}

// 从vector构造多项式
PolynomialMap::PolynomialMap(const vector<int>& deg, const vector<double>& cof) {
    auto itd = deg.begin();
    auto itc = cof.begin();
    while (itd != deg.end() && itc != cof.end()) {
        if (*itc != 0.0) {
            m_Polynomial[*itd] += *itc;
            if (m_Polynomial[*itd] == 0.0) {
                m_Polynomial.erase(*itd);
            }
        }
        ++itd; ++itc;
    }
}

// 获取指定次数的系数（const版本）
double PolynomialMap::coff(int i) const {
    auto it = m_Polynomial.find(i);
    return (it != m_Polynomial.end()) ? it->second : 0.0;
}

// 获取指定次数的系数（非const版本）
double& PolynomialMap::coff(int i) {
    return m_Polynomial[i]; // 自动插入新项（系数初始化为0）
}

// 删除零系数项
void PolynomialMap::compress() {
    auto it = m_Polynomial.begin();
    while (it != m_Polynomial.end()) {
        if (it->second == 0.0) {
            it = m_Polynomial.erase(it);
        } else {
            ++it;
        }
    }
}

// 多项式加法
PolynomialMap PolynomialMap::operator+(const PolynomialMap& right) const {
    PolynomialMap result(*this);
    for (const auto& term : right.m_Polynomial) {
        result.m_Polynomial[term.first] += term.second;
        if (result.m_Polynomial[term.first] == 0.0) {
            result.m_Polynomial.erase(term.first);
        }
    }
    return result;
}

// 多项式减法
PolynomialMap PolynomialMap::operator-(const PolynomialMap& right) const {
    PolynomialMap result(*this);
    for (const auto& term : right.m_Polynomial) {
        result.m_Polynomial[term.first] -= term.second;
        if (result.m_Polynomial[term.first] == 0.0) {
            result.m_Polynomial.erase(term.first);
        }
    }
    return result;
}

// 多项式乘法
PolynomialMap PolynomialMap::operator*(const PolynomialMap& right) const {
    PolynomialMap result;
    for (const auto& t1 : m_Polynomial) {
        for (const auto& t2 : right.m_Polynomial) {
            int deg = t1.first + t2.first;
            double cof = t1.second * t2.second;
            result.m_Polynomial[deg] += cof;
            if (result.m_Polynomial[deg] == 0.0) {
                result.m_Polynomial.erase(deg);
            }
        }
    }
    return result;
}

// 赋值运算符
PolynomialMap& PolynomialMap::operator=(const PolynomialMap& right) {
    if (this != &right) {
        m_Polynomial = right.m_Polynomial;
    }
    return *this;
}

// 打印多项式
void PolynomialMap::Print() const {
    bool first = true;
    for (const auto& term : m_Polynomial) {
        if (!first) cout << " + ";
        cout << term.second << "x^" << term.first;
        first = false;
    }
    if (first) cout << "0";
    cout << endl;
}

// 从文件读取多项式
bool PolynomialMap::ReadFromFile(const string& file) {
    ifstream fin(file);
    if (!fin) return false;

    m_Polynomial.clear();
    char type;
    int n;
    fin >> type >> n; // 读取首行标识 P 和项数

    for (int i = 0; i < n; ++i) {
        int deg;
        double cof;
        fin >> deg >> cof;
        if (cof != 0.0) {
            m_Polynomial[deg] += cof;
            if (m_Polynomial[deg] == 0.0) {
                m_Polynomial.erase(deg);
            }
        }
    }
    fin.close();
    return true;
}