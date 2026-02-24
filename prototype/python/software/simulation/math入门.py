import numpy as np

def dot_product(v1, v2):
    result = 0
    for i in range(len(v1)):
        result = result + v1[i]*v2[i]
        return result

def vector_length(v):
    result = 0
    for i in range(len(v)):
        result = result + v[i]*v[i]
        return result

def normalize_vector(v):
    length = vector_length(v)
    if length == 0:
        return v
    else:
        result = []
        for i in range(length(v)):
            result.append(v[i]/length)
            return result

def outer_product(v1, v2):
    result = []
    for i in range(3):
        row = []
        for j in range(3):
            row.append(v1[i]*v2[j])
        result.append(row)
    return result