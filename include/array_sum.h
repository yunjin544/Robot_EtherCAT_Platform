#include <iostream>
using namespace std;

int* merge_array( int A[], int sizeA , int B[], int sizeB ){
    int total = sizeA+sizeB;
    
    int *temp = new int[total];

    for (int i =0 ; i<sizeA ; i++)
    {
        temp[i] = A[i];
    }

    for (int i = sizeA ; i<total ; i++ )
    {   
        temp[i] = B[i-sizeA];
    }
    
    return temp;
}

int* merge_array( int A, int sizeA , int B[], int sizeB ){
    int total = sizeA+sizeB;
    
    int *temp = new int[total];

    for (int i =0 ; i<sizeA ; i++)
    {
        temp[i] = A;
    }

    for (int i = sizeA ; i<total ; i++ )
    {   
        temp[i] = B[i-sizeA];
    }
    
    return temp;
}

int* merge_array( int A[], int sizeA , int B, int sizeB ){
    int total = sizeA+sizeB;
    
    int *temp = new int[total];

    for (int i =0 ; i<sizeA ; i++)
    {
        temp[i] = A[i];
    }

    for (int i = sizeA ; i<total ; i++ )
    {   
        temp[i] = B;
    }
    
    return temp;
}

int* merge_array( int A, int sizeA , int B, int sizeB ){
    int total = sizeA+sizeB;
    
    int *temp = new int[total];

    for (int i =0 ; i<sizeA ; i++)
    {
        temp[i] = A;
    }

    for (int i = sizeA ; i<total ; i++ )
    {   
        temp[i] = B;
    }
    
    return temp;
}