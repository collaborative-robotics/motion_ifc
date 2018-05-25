#include <motion_ifc/Communication.h>


boost::shared_ptr<ros::NodeHandle> CommunicationBase::node;

///
/// \brief CommunicationBase::CommunicationBase
///
CommunicationBase::CommunicationBase():_is_data_new(false){
    init();
}

///
/// \brief CommunicationBase::init
///
void CommunicationBase::init(){
    int argc = 0;
    char ** argv;
    ros::init(argc, argv, "motion_ifc");
    node.reset(new ros::NodeHandle);
}


template <typename D>
///
/// \brief Communication<D>::Communication
/// \param topic_name
/// \param com_dir
/// \param wd_timeout
///
Communication<D>::Communication(string topic_name, CommDirection com_dir, double wd_timeout){
    if (com_dir == OUTGOING){
        pub = CommunicationBase::node->advertise<D>(topic_name, 10);
    }
    else if(com_dir == INCOMING ){
        set_wd_time_out(wd_timeout);
        sub = CommunicationBase::node->subscribe(topic_name, 10, &Communication<D>::cb, this);
    }
}

template <typename D>
///
/// \brief Communication<D>::cb
/// \param data
///
void Communication<D>::cb(const boost::shared_ptr<const D> &data){
    time_stamp = ros::Time::now().toSec();
    acknowledge_wd();
    cb_data = *data;
    _is_data_new = true;
}

template<typename D>
///
/// \brief Communication<D>::get_data
/// \param data
///
void Communication<D>::get_data(D &data){
    data = cb_data;
    _is_data_new = false;
}

template<typename D>
///
/// \brief Communication<D>::set_data
/// \param data
///
void Communication<D>::set_data(D& data){
    pub.publish(data);
}

template<typename D>
///
/// \brief Communication<D>::execute_controller
///
void Communication<D>::execute_controller(){
    if(_is_data_new){
        (*command_method)(cb_data);
        _is_data_new = false;
    }
}


///
/// \brief CommunicationIfc::CommunicationIfc
///
CommunicationIfc::CommunicationIfc(){

}

///
/// \brief CommunicationIfc::create_communication_interface
/// \param topic_name
/// \param com_dir
/// \param wd_timeout
/// \return
///
CommBasePtr CommunicationIfc::create_communication_interface(std::string topic_name, CommDirection com_dir, double wd_timeout){
    std::vector<std::string> x = split_str(topic_name, '/');
    std::vector<std::string> crtk_str = split_str(x.back(), '_');
    char op_space = crtk_str[1][0];
    char controller = crtk_str[1][1];

    std::cout << topic_name << std::endl;

    CommBasePtr commBase;
    if (op_space == 'c'){
        switch (controller){
        case 'p':
            commBase = CommBasePtr(new Communication<_cp_data_type>(topic_name, com_dir, wd_timeout));
            break;
        case 'r':
            commBase = CommBasePtr(new Communication<_cr_data_type>(topic_name, com_dir, wd_timeout));
            break;
        case 'v':
            commBase = CommBasePtr(new Communication<_cv_data_type>(topic_name, com_dir, wd_timeout));
            break;
        case 'f':
            commBase = CommBasePtr(new Communication<_cf_data_type>(topic_name, com_dir, wd_timeout));
            break;
        default:
            throw "The specified format isn't understood";
        }
    }
    else if (op_space == 'j'){
        switch (controller){
        case 'p':
            commBase = CommBasePtr(new Communication<_jp_data_type>(topic_name, com_dir, wd_timeout));
            break;
        case 's':
            commBase = CommBasePtr(new Communication<_js_data_type>(topic_name, com_dir, wd_timeout));
            break;
        case 'r':
            commBase = CommBasePtr(new Communication<_jr_data_type>(topic_name, com_dir, wd_timeout));
            break;
        case 'v':
            commBase = CommBasePtr(new Communication<_jv_data_type>(topic_name, com_dir, wd_timeout));
            break;
        case 'f':
            commBase = CommBasePtr(new Communication<_jf_data_type>(topic_name, com_dir, wd_timeout));
            break;
        default:
            throw "The specified format isn't understood";
        }

    }
    return commBase;
}
