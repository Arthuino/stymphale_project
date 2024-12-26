import numpy as np
import sys
# Define your custom type
class Complex32:
    def __init__(self, real, imag):
        self.real = np.float16(real)
        self.imag = np.float16(imag)

    def __repr__(self):
        # This method defines how the object is represented when printed
        return f"({self.real} + {self.imag}j)"

    @staticmethod
    def complex64_to_complex32(complex64_number):
        # Ensure the input is complex64
        if isinstance(complex64_number, (np.complex64,complex)):
            return Complex32(complex64_number.real, complex64_number.imag)
        else:
            raise ValueError(f"Input must be of type complex64 ({type(complex64_number)})")

    @staticmethod
    def complex32_to_complex64(complex32_number):
        if isinstance(complex32_number, Complex32):
            return np.complex64(complex32_number.real, complex32_number.imag)
        else:
            raise ValueError(f"Input must be of type complex32 ({type(complex32_number)})")

if __name__ == '__main__':  
    complex_num = np.complex64(11355 + 24524.45j)  # A complex64 number
    print("complex_num")
    print(complex_num)
    print(sys.getsizeof(complex_num))
    print(type(complex_num.real))


    light_complex = Complex32.complex64_to_complex32(complex_num)
    print("light_complex")
    print(light_complex)
    print(sys.getsizeof(light_complex))
    print(type(light_complex.real))


    heavy_complex = Complex32.complex32_to_complex64(light_complex)
    print("heavy_complex")
    print(heavy_complex)
    print(sys.getsizeof(heavy_complex))
    print(type(heavy_complex.real))


    # Set the dimensions of the 3D matrix (e.g., 3x3x3)
    depth, rows, cols = 3, 3, 3

    # Generate random real and imaginary parts separately
    real_part = np.random.randn(depth, rows, cols).astype(np.float32)  # Real part
    imag_part = np.random.randn(depth, rows, cols).astype(np.float32)  # Imaginary part

    # Combine the real and imaginary parts into a complex number
    complex_matrix = real_part + 1j * imag_part

    # Ensure the dtype is complex64
    complex_matrix = complex_matrix.astype(np.complex64)

    print(complex_matrix.shape)

    func = np.vectorize(Complex32.complex64_to_complex32)

    compressed_matrix = func(complex_matrix)

    print(compressed_matrix)

    decompress_matrix = np.vectorize(Complex32.complex32_to_complex64)(compressed_matrix)




