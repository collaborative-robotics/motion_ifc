#ifndef FCN_HANDLE_H
#define FCN_HANDLE_H

class FcnHandleBase{
public:
    FcnHandleBase(){}
    void set();
    void operator()(void);
};

template<typename D>
class FcnHandle: public FcnHandleBase{
public:
    FcnHandle():_is_set(false){}
    ~FcnHandle(){}

    template<typename C>
    void assign_fcn(void (C::*conversion_fcn)(D&), C *obj){
        fcn_handle = boost::bind(conversion_fcn, obj, _1);
        _is_set = true;
    }

    void operator()(D&data){
        std::cout << "IMAPOTATO" << std::endl;
        fcn_handle(data);
    }

    boost::function<void (D&)> fcn_handle;
    double idx;
    bool _is_set = false;
};


#endif
