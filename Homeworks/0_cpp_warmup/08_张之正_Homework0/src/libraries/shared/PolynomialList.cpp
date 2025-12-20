#include "PolynomialList.h"
#include <fstream>
#include <algorithm>
#include <iterator>
#include <iostream>

using namespace std;

// 拷贝构造函数
PolynomialList::PolynomialList(const PolynomialList& other) 
    : m_Polynomial(other.m_Polynomial) {}

// 从文件构造多项式
PolynomialList::PolynomialList(const string& file) {
    if (!ReadFromFile(file)) {
        throw runtime_error("Failed to read polynomial from file");
    }
    compress();
}

// 从数组构造多项式
PolynomialList::PolynomialList(const double* cof, const int* deg, int n) {
    for (int i = 0; i < n; ++i) {
        AddOneTerm(Term(deg[i], cof[i]));
    }
    compress();
}

// 从vector构造多项式
PolynomialList::PolynomialList(const vector<int>& deg, const vector<double>& cof) {
    auto itd = deg.begin();
    auto itc = cof.begin();
    while (itd != deg.end() && itc != cof.end()) {
        AddOneTerm(Term(*itd, *itc));
        ++itd; ++itc;
    }
    compress();
}

// 获取指定次数的系数（const版本）
double PolynomialList::coff(int i) const {
    for (const auto& term : m_Polynomial) {
        if (term.deg == i) return term.cof;
        if (term.deg < i) break; // 列表按降序排列
    }
    return 0.0;
}

// 获取指定次数的系数（非const版本）
double& PolynomialList::coff(int i) {
    for (auto& term : m_Polynomial) {
        if (term.deg == i) return term.cof;
        if (term.deg < i) break;
    }
    // 插入新项并返回引用
    return AddOneTerm(Term(i, 0.0)).cof;
}

// 合并同类项并删除零系数项
void PolynomialList::compress() {
    m_Polynomial.remove_if([](const Term& t) { return t.cof == 0.0; });
    m_Polynomial.sort([](const Term& a, const Term& b) { return a.deg > b.deg; });
}

// 多项式加法
PolynomialList PolynomialList::operator+(const PolynomialList& right) const {
    PolynomialList result(*this);
    for (const auto& term : right.m_Polynomial) {
        result.AddOneTerm(term);
    }
    result.compress();
    return result;
}

// 多项式减法
PolynomialList PolynomialList::operator-(const PolynomialList& right) const {
    PolynomialList result(*this);
    for (const auto& term : right.m_Polynomial) {
        result.AddOneTerm(Term(term.deg, -term.cof));
    }
    result.compress();
    return result;
}

// 多项式乘法
PolynomialList PolynomialList::operator*(const PolynomialList& right) const {
    PolynomialList result;
    for (const auto& t1 : m_Polynomial) {
        for (const auto& t2 : right.m_Polynomial) {
            int deg = t1.deg + t2.deg;
            double cof = t1.cof * t2.cof;
            result.AddOneTerm(Term(deg, cof));
        }
    }
    result.compress();
    return result;
}

// 赋值运算符
PolynomialList& PolynomialList::operator=(const PolynomialList& right) {
    if (this != &right) {
        m_Polynomial = right.m_Polynomial;
    }
    return *this;
}

// 打印多项式
void PolynomialList::Print() const {
    bool first = true;
    for (const auto& term : m_Polynomial) {
        if (!first) cout << " + ";
        cout << term.cof << "x^" << term.deg;
        first = false;
    }
    if (first) cout << "0";
    cout << endl;
}

// 从文件读取多项式
bool PolynomialList::ReadFromFile(const string& file) {
    ifstream fin(file);
    if (!fin) return false;

    char type;
    int n;
    fin >> type >> n; // 读取首行标识 P 和项数

    for (int i = 0; i < n; ++i) {
        int deg;
        double cof;
        fin >> deg >> cof;
        AddOneTerm(Term(deg, cof));
    }
    fin.close();
    return true;
}

PolynomialList::Term& PolynomialList::AddOneTerm(const Term& term) {
    if (term.cof == 0.0) {
        static Term dummy(0, 0.0);
        return dummy; // 零系数项不插入
    }

    auto it = m_Polynomial.begin();
    while (it != m_Polynomial.end()) {
        if (it->deg == term.deg) {
            it->cof += term.cof;
            if (it->cof == 0.0) {
                it = m_Polynomial.erase(it); // 删除项后返回 dummy
                static Term dummy(0, 0.0);
                return dummy;
            } else {
                return *it; // 返回合并后的有效项
            }
        } else if (it->deg < term.deg) {
            it = m_Polynomial.insert(it, term);
            return *it; // 返回新插入的项
        }
        ++it;
    }
    // 插入到末尾
    m_Polynomial.push_back(term);
    return m_Polynomial.back();
}