class FcnHandleBase{
public:
    FcnHandleBase(){}
    void set();
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

    void set(D &data){
        fcn_handle(data);
    }

    boost::function<void (D&)> fcn_handle;
    double idx;
    bool _is_set = false;
};
