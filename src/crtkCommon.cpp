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

///
/// \brief Controllers::_name_space
///
std::string crtk::_name_space = "dvrk";

///
/// \brief Controllers::_arm_name
///
std::string crtk::_arm_name = "MTMR";

///
/// \brief crtk::crtk
///
crtk::crtk(){

}

///
/// \brief crtk::init
/// \param name_space
/// \param arm_name
///
void crtk::set_ns_and_arm(std::string name_space, std::string arm_name){
    _name_space = name_space;
    _arm_name = arm_name;
}

///
/// \brief Controllers::get_topic_prefix
/// \return
///
std::string crtk::get_name_space(){
    return _name_space;
}

std::string crtk::get_arm_name(){
    return _arm_name;
}

