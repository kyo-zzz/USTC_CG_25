// DArray.cpp
#include "DArray.h"
#include <iostream>
#include <stdexcept>
#include <algorithm>

// 默认构造函数
DArray::DArray() {
    Init();
}

// 带初始大小和默认值的构造函数
DArray::DArray(int nSize, double dValue) {
    if (nSize < 0) throw std::invalid_argument("Invalid size");
    m_nMax = (nSize == 0) ? 0 : nSize;
    m_nSize = nSize;
    m_pData = (m_nMax > 0) ? new double[m_nMax] : nullptr;
    std::fill(m_pData, m_pData + m_nSize, dValue);
}

// 拷贝构造函数
DArray::DArray(const DArray& arr) {
    m_nMax = arr.m_nMax;
    m_nSize = arr.m_nSize;
    m_pData = (m_nMax > 0) ? new double[m_nMax] : nullptr;
    std::copy(arr.m_pData, arr.m_pData + m_nSize, m_pData);
}

// 析构函数
DArray::~DArray() {
    Free();
}

// 初始化成员变量
void DArray::Init() {
    m_pData = nullptr;
    m_nSize = 0;
    m_nMax = 0;
}

// 释放内存
void DArray::Free() {
    delete[] m_pData;
    Init();
}

// 预分配内存 (核心优化逻辑)
void DArray::Reserve(int nCapacity) {
    if (nCapacity <= m_nMax) return;

    double* newData = new double[nCapacity];
    if (m_nSize > 0) {
        std::copy(m_pData, m_pData + m_nSize, newData);
    }
    delete[] m_pData;
    m_pData = newData;
    m_nMax = nCapacity;
}

// 打印数组元素
void DArray::Print() const {
    if (m_nSize == 0) {
        std::cout << "Empty array" << std::endl;
        return;
    }
    std::cout << "Array(" << m_nSize << "/" << m_nMax << "): ";
    for (int i = 0; i < m_nSize; ++i) {
        std::cout << m_pData[i];
        if (i != m_nSize - 1) std::cout << ", ";
    }
    std::cout << std::endl;
}

// 获取数组大小
int DArray::GetSize() const {
    return m_nSize;
}

// 设置数组大小
void DArray::SetSize(int nSize) {
    if (nSize < 0) throw std::invalid_argument("Invalid size");
    
    if (nSize > m_nMax) {
        Reserve(nSize);  // 扩展容量
    }
    m_nSize = nSize;
}

// 获取元素 (const)
const double& DArray::GetAt(int nIndex) const {
    if (nIndex < 0 || nIndex >= m_nSize) {
        throw std::out_of_range("Index out of range");
    }
    return m_pData[nIndex];
}

// 设置元素值
void DArray::SetAt(int nIndex, double dValue) {
    if (nIndex < 0 || nIndex >= m_nSize) {
        throw std::out_of_range("Index out of range");
    }
    m_pData[nIndex] = dValue;
}

// 下标运算符 (non-const)
double& DArray::operator[](int nIndex) {
    if (nIndex < 0 || nIndex >= m_nSize) {
        throw std::out_of_range("Index out of range");
    }
    return m_pData[nIndex];
}

// 下标运算符 (const)
const double& DArray::operator[](int nIndex) const {
    if (nIndex < 0 || nIndex >= m_nSize) {
        throw std::out_of_range("Index out of range");
    }
    return m_pData[nIndex];
}

// 尾部追加元素 (优化后)
void DArray::PushBack(double dValue) {
    if (m_nSize >= m_nMax) {
        Reserve(m_nMax == 0 ? 1 : m_nMax * 2);
    }
    m_pData[m_nSize++] = dValue;
}

// 删除指定位置元素
void DArray::DeleteAt(int nIndex) {
    if (nIndex < 0 || nIndex >= m_nSize) return;

    // 移动后续元素
    for (int i = nIndex; i < m_nSize - 1; ++i) {
        m_pData[i] = m_pData[i + 1];
    }
    --m_nSize;
}

// 插入元素到指定位置 (优化后)
void DArray::InsertAt(int nIndex, double dValue) {
    if (nIndex < 0 || nIndex > m_nSize) {
        throw std::out_of_range("Index out of range");
    }

    if (m_nSize >= m_nMax) {
        Reserve(m_nMax == 0 ? 1 : m_nMax * 2);
    }

    // 移动元素腾出空间
    for (int i = m_nSize; i > nIndex; --i) {
        m_pData[i] = m_pData[i - 1];
    }
    m_pData[nIndex] = dValue;
    ++m_nSize;
}

// 赋值运算符
DArray& DArray::operator=(const DArray& arr) {
    if (this != &arr) {
        delete[] m_pData;
        m_nMax = arr.m_nMax;
        m_nSize = arr.m_nSize;
        m_pData = (m_nMax > 0) ? new double[m_nMax] : nullptr;
        std::copy(arr.m_pData, arr.m_pData + m_nSize, m_pData);
    }
    return *this;
}