#include <dynamics_plugin.h>

namespace gazebo {

    DynamicsPlugin::~DynamicsPlugin() {}

    void DynamicsPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        ROS_INFO("Loading the dynamics plugin...");
        model_ = _parent;
        world_ = model_->GetWorld();
        
        // Load plugin parameters
        if (_sdf->HasElement("waterLevel"))
            water_level_ = _sdf->GetElement("waterLevel")->Get<double>();
        else
            gzerr << "[dynamics_plugin] Please specify the waterLevel parameter.\n";

        if (_sdf->HasElement("waterDensity"))
            water_density_ = _sdf->GetElement("waterDensity")->Get<double>();
        else
            gzerr << "[dynamics_plugin] Please specify the waterDensity parameter.\n";

        if (_sdf->HasElement("xDotU"))
            param_X_dot_u_ = _sdf->GetElement("xDotU")->Get<double>();
        else
            gzerr << "[dynamics_plugin] Please specify the yDotV parameter.\n";

        if (_sdf->HasElement("yDotV"))
            param_Y_dot_v_ = _sdf->GetElement("yDotV")->Get<double>();
        else
            gzerr << "[dynamics_plugin] Please specify the yDotV parameter.\n";
        
        if (_sdf->HasElement("nDotR"))
            param_N_dot_r_ = _sdf->GetElement("nDotR")->Get<double>();
        else
            gzerr << "[dynamics_plugin] Please specify the nDotR parameter.\n";
        
        if (_sdf->HasElement("xU"))
            param_X_u_ = _sdf->GetElement("xU")->Get<double>();
        else
            gzerr << "[dynamics_plugin] Please specify the xU parameter.\n";

        if (_sdf->HasElement("xUU"))
            param_X_uu_ = _sdf->GetElement("xUU")->Get<double>();
        else
            gzerr << "[dynamics_plugin] Please specify the xUU parameter.\n";

        if (_sdf->HasElement("yV"))
            param_Y_v_ = _sdf->GetElement("yV")->Get<double>();
        else
            gzerr << "[dynamics_plugin] Please specify the yV parameter .\n";

        if (_sdf->HasElement("yVV"))
            param_Y_vv_ = _sdf->GetElement("yVV")->Get<double>();
        else
            gzerr << "[dynamics_plugin] Please specify the yVV parameter.\n";
        
        if (_sdf->HasElement("zW"))
            param_Z_w_ = _sdf->GetElement("zW")->Get<double>();
        else
            gzerr << "[dynamics_plugin] Please specify the zW parameter.\n";
        
        if (_sdf->HasElement("kP"))
            param_K_p_ = _sdf->GetElement("kP")->Get<double>();
        else
            gzerr << "[dynamics_plugin] Please specify the kP parameter.\n";

        if (_sdf->HasElement("mQ"))
            param_M_q_ = _sdf->GetElement("mQ")->Get<double>();
        else
            gzerr << "[dynamics_plugin] Please specify the mQ parameter.\n";

        if (_sdf->HasElement("nR"))
            param_N_r_ = _sdf->GetElement("nR")->Get<double>();
        else
            gzerr << "[dynamics_plugin] Please specify the nR parameter.\n";

        if (_sdf->HasElement("nRR"))
            param_N_rr_ = _sdf->GetElement("nRR")->Get<double>();
        else
            gzerr << "[dynamics_plugin] Please specify the nRR parameter.\n";

        if (_sdf->HasElement("boatWidth"))
            param_boat_width_ = _sdf->GetElement("boatWidth")->Get<double>();
        else
            gzerr << "[dynamics_plugin] Please specify the boatWidth parameter.\n";

        if (_sdf->HasElement("boatLength"))
            param_boat_length_ = _sdf->GetElement("boatLength")->Get<double>();
        else
            gzerr << "[dynamics_plugin] Please specify the boatLength parameter.\n";

        if (_sdf->HasElement("metacentricLength"))
            param_metacentric_length_ = _sdf->GetElement("metacentricLength")->Get<double>();
        else
            gzerr << "[dynamics_plugin] Please specify the metacentricLength parameter.\n";

        if (_sdf->HasElement("metacentricWidth"))
            param_metacentric_width_ = _sdf->GetElement("metacentricWidth")->Get<double>();
        else
            gzerr << "[dynamics_plugin] Please specify the metacentricWidth parameter.\n";

        if (_sdf->HasElement("boatArea"))
            param_boat_area_ = _sdf->GetElement("boatArea")->Get<double>();
        else
            gzerr << "[dynamics_plugin] Please specify the boatArea parameter.\n";


        //  Enumerating model
        ROS_INFO_STREAM("Enumerating Model...");
        ROS_INFO_STREAM("Model name = "<< model_->GetName());

        physics::Link_V links = model_->GetLinks();
        for (unsigned int i = 0; i < links.size(); i++){
            ROS_INFO_STREAM("Link: "<< links[i]->GetName());
        }

        // Applying force to the base link
        link_ = model_->GetLink(links[0]->GetName());
        









        // Listen to the update event. This event is broadcast every simulation iteration.
        updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&DynamicsPlugin::OnUpdate, this));
    }




    void DynamicsPlugin::OnUpdate()
    {        
       // TODO



    }

    GZ_REGISTER_MODEL_PLUGIN(DynamicsPlugin);
}