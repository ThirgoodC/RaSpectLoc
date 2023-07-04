
#include <cstring>

class String {
    public:
        // Constructor
        String(const char* str) {
            size_t len = strlen(str);
            data_ = new char[len + 1];
            strcpy(data_, str);
            length_ = len;
        }

        // Copy constructor
        String(const String& other) {
            size_t len = other.length_;
            data_ = new char[len + 1];
            strcpy(data_, other.data_);
            length_ = len;
        }

        // Destructor
        ~String() {
            delete[] data_;
        }

        // Assignment operator
        String& operator=(const String& other) {
            if (this != &other) {
                delete[] data_;
                size_t len = other.length_;
                data_ = new char[len + 1];
                strcpy(data_, other.data_);
                length_ = len;
            }
            return *this;
        }

        // Returns the length of the string
        size_t length() const {
            return length_;
        }

        // In-place multiplication operator
        String& operator*=(int i) {
            if (i <= 0) {
                // if i is negative or zero, set the string to an empty string
                delete[] data_;
                data_ = new char[1];
                data_[0] = '\0';
                length_ = 0;
            }
            else {
                // create a new string with the duplicated contents
                size_t new_len = length_ * i;
                char* new_data = new char[new_len + 1];
                for (int j = 0; j < i; ++j) {
                    strcpy(new_data + j * length_, data_);
                }

                // replace the existing string with the new one
                delete[] data_;
                data_ = new_data;
                length_ = new_len;
            }
            return *this;
        }
        
        // Function to count the occurrences of a character in a StringC object
        size_t countOccurrences(char c) {
            size_t count = 0;
            for (size_t i = 0; i < length(); ++i) {
                if (data_[i] == c) {
                    ++count;
                }
            }
            return count;
        }

        // Returns the character at the specified index
        char operator[](size_t index) const {
            return data_[index];
        }
        
        // Non-in-place multiplication operator
        friend String operator*(const String& str, int i) {
            if (i <= 0) {
                // if i is negative or zero, return an empty string
                return String("");
            }
            else {
                // create a new string with the duplicated contents
                size_t new_len = str.length_ * i;
                char* new_data = new char[new_len + 1];
                for (int j = 0; j < i; ++j) {
                    strcpy(new_data + j * str.length_, str.data_);
                }
                return String(new_data);
            }
        }

    private:
        char* data_;
        size_t length_;
};

int main(int argc, char** argv)
{
    String s1("hello");
    String s2 = s1 * 3; // s2 contains "hellohellohello"
}