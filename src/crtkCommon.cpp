#include <motion_ifc/crtkCommon.h>

// Copied function from:
// https://stackoverflow.com/questions/236129/the-most-elegant-way-to-iterate-the-words-of-a-string

template<typename Out>
///
/// \brief split_str
/// \param s
/// \param delim
/// \param result
///
void split_str(const std::string &s, char delim, Out result) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        *(result++) = item;
    }
}

///
/// \brief split_str
/// \param s
/// \param delim
/// \return
///
std::vector<std::string> split_str(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split_str(s, delim, std::back_inserter(elems));
    return elems;
}

