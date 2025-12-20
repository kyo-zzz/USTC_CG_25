// DArray.cpp
#include "DArray.h"
#include <iostream>
#include <stdexcept>
#include <algorithm> // for std::copy

// Default constructor
DArray::DArray() {
    Init();
}

// Constructor with size and default value
DArray::DArray(int nSize, double dValue) {
    if (nSize <= 0) {
        Init();
        return;
    }
    m_nSize = nSize;
    m_pData = new double[m_nSize];
    for (int i = 0; i < m_nSize; ++i) {
        m_pData[i] = dValue;
    }
}

// Copy constructor
DArray::DArray(const DArray& arr) {
    if (arr.m_nSize == 0) {
        Init();
    } else {
        m_nSize = arr.m_nSize;
        m_pData = new double[m_nSize];
        std::copy(arr.m_pData, arr.m_pData + m_nSize, m_pData);
    }
}

// Destructor
DArray::~DArray() {
    Free();
}

// Initialize array
void DArray::Init() {
    m_pData = nullptr;
    m_nSize = 0;
}

// Free memory
void DArray::Free() {
    delete[] m_pData;
    m_pData = nullptr;
    m_nSize = 0;
}

// Print array elements
void DArray::Print() const {
    if (m_nSize == 0) {
        std::cout << "Empty array" << std::endl;
        return;
    }
    std::cout << "Array elements: ";
    for (int i = 0; i < m_nSize; ++i) {
        std::cout << m_pData[i];
        if (i != m_nSize - 1) std::cout << ", ";
    }
    std::cout << std::endl;
}

// Get array size
int DArray::GetSize() const {
    return m_nSize;
}

// Set array size
void DArray::SetSize(int nSize) {
    if (nSize < 0) throw std::invalid_argument("Invalid size");
    if (nSize == m_nSize) return;
    
    double* newData = new double[nSize]{};
    int copySize = std::min(nSize, m_nSize);
    if (copySize > 0) {
        std::copy(m_pData, m_pData + copySize, newData);
    }
    delete[] m_pData;
    m_pData = newData;
    m_nSize = nSize;
}

// Get element (const)
const double& DArray::GetAt(int nIndex) const {
    if (nIndex < 0 || nIndex >= m_nSize) {
        throw std::out_of_range("Index out of range");
    }
    return m_pData[nIndex];
}

// Set element
void DArray::SetAt(int nIndex, double dValue) {
    if (nIndex < 0 || nIndex >= m_nSize) {
        throw std::out_of_range("Index out of range");
    }
    m_pData[nIndex] = dValue;
}

// Operator[] (non-const)
double& DArray::operator[](int nIndex) {
    if (nIndex < 0 || nIndex >= m_nSize) {
        throw std::out_of_range("Index out of range");
    }
    return m_pData[nIndex];
}

// Operator[] (const)
const double& DArray::operator[](int nIndex) const {
    if (nIndex < 0 || nIndex >= m_nSize) {
        throw std::out_of_range("Index out of range");
    }
    return m_pData[nIndex];
}

// Add element to end
void DArray::PushBack(double dValue) {
    double* newData = new double[m_nSize + 1];
    std::copy(m_pData, m_pData + m_nSize, newData);
    newData[m_nSize] = dValue;
    delete[] m_pData;
    m_pData = newData;
    ++m_nSize;
}

// Delete element at index
void DArray::DeleteAt(int nIndex) {
    if (nIndex < 0 || nIndex >= m_nSize) return;
    
    double* newData = new double[m_nSize - 1];
    std::copy(m_pData, m_pData + nIndex, newData);
    std::copy(m_pData + nIndex + 1, m_pData + m_nSize, newData + nIndex);
    delete[] m_pData;
    m_pData = newData;
    --m_nSize;
}

// Insert element at index
void DArray::InsertAt(int nIndex, double dValue) {
    if (nIndex < 0 || nIndex > m_nSize) {
        throw std::out_of_range("Index out of range");
    }
    
    double* newData = new double[m_nSize + 1];
    std::copy(m_pData, m_pData + nIndex, newData);
    newData[nIndex] = dValue;
    std::copy(m_pData + nIndex, m_pData + m_nSize, newData + nIndex + 1);
    delete[] m_pData;
    m_pData = newData;
    ++m_nSize;
}

// Assignment operator
DArray& DArray::operator=(const DArray& arr) {
    if (this != &arr) {
        delete[] m_pData;
        m_nSize = arr.m_nSize;
        if (m_nSize > 0) {
            m_pData = new double[m_nSize];
            std::copy(arr.m_pData, arr.m_pData + m_nSize, m_pData);
        } else {
            m_pData = nullptr;
        }
    }
    return *this;
}