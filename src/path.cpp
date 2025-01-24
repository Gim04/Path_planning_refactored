#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <vector>
#include <iostream>

#include "std_msgs/msg/string.hpp"
#include "zed_msgs/msg/cones.hpp"


#include <rclcpp/rclcpp.hpp>

#include <control_msgs/msg/waypoint_array_stamped.hpp>
#include <control_msgs/msg/ref_data.hpp>
#include <control_msgs/msg/waypoint.hpp>
#include <zed_msgs/msg/cone.hpp>
#include <zed_msgs/msg/cones.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <path_interfaces/msg/pubxy.hpp>
#include <path_interfaces/msg/waypoints_plus_cones.hpp>

//#include <Eigen/Dense>
#include <algorithm>
#include <glm/glm.hpp>

#include "CDT.h" //delaunay triangulation library
#include "path.h"

using namespace std::placeholders;

class LoadDataNode : public rclcpp::Node {
public:
    LoadDataNode()
        : Node("load_data_node") {
        RCLCPP_INFO(this->get_logger(), "Node has started");

        // Declare parameters
        this->declare_parameter<double>("Look_ahead_value", 0.0);
        this->declare_parameter<int>("search_region_value", 0);
        this->declare_parameter<int>("eval_length_value", 0);
        this->declare_parameter<int>("numWaypoints_to_spline_value", 0);
        this->declare_parameter<double>("Max_Velocity_value", 0.0);
        this->declare_parameter<double>("Min_Velocity_value", 0.0);
        this->declare_parameter<int>("Max_curvature_value", 0);
        this->declare_parameter<int>("Spline_curvature_constrain_value", 0);
        this->declare_parameter<double>("Cones_filter_param_distance", 0.0);
        this->declare_parameter<double>("Cones_filter_param_y", 0.0);
        this->declare_parameter<int>("Max_spline_degree", 0);

        // Get parameters
        leng_ = this->get_parameter("Look_ahead_value").as_double();
        search_region_ = this->get_parameter("search_region_value").as_int();
        eval_length_ = this->get_parameter("eval_length_value").as_int();
        numWaypoints_to_spline_ = this->get_parameter("numWaypoints_to_spline_value").as_int();
        spline_curvature_constrain_ = this->get_parameter("Spline_curvature_constrain_value").as_int();
        Vmax_ = this->get_parameter("Max_Velocity_value").as_double();
        Vmin_ = this->get_parameter("Min_Velocity_value").as_double();
        RCmax_ = this->get_parameter("Max_curvature_value").as_int();
        filter_param_distance_ = this->get_parameter("Cones_filter_param_distance").as_double();
        filter_param_y_ = this->get_parameter("Cones_filter_param_y").as_double();
        max_spline_degree_ = this->get_parameter("Max_spline_degree").as_int();

        waypoint_threshold_ = 6;
        special_case_ = 0;
        spline_degree_ = 1;
        SpecialActivated_ = false;
        la_dyn_ = (leng_ == 0.0);
        pred_eval_length_ = 20;
        avg_curvature_ = 5.0;
        arclen_minore_ = false;

        RCLCPP_INFO(this->get_logger(), "----- PARAMETRI PATH AUTOCROSS/TRACKDRIVE -----");
        if (la_dyn_) {
            RCLCPP_INFO(this->get_logger(), "Look Ahead dinamica");
        } else {
            RCLCPP_INFO(this->get_logger(), "Look Ahead statica: %f", leng_);
        }
        RCLCPP_INFO(this->get_logger(), "Search Region: %d", search_region_);
        RCLCPP_INFO(this->get_logger(), "Eval Length: %d", eval_length_);
        RCLCPP_INFO(this->get_logger(), "Numero waypoints to spline: %d", numWaypoints_to_spline_);
        RCLCPP_INFO(this->get_logger(), "Max Velocity: %f", Vmax_);
        RCLCPP_INFO(this->get_logger(), "Min Velocity: %f", Vmin_);
        RCLCPP_INFO(this->get_logger(), "Max Curvature: %d", RCmax_);
        RCLCPP_INFO(this->get_logger(), "Spline Curvature Constrain Value: %d", spline_curvature_constrain_);
        RCLCPP_INFO(this->get_logger(), "Max distance cones filter: %f", filter_param_distance_);
        RCLCPP_INFO(this->get_logger(), "Max Y cones filter: %f", filter_param_y_);

        // Subscribers
        subscription_cones_ = this->create_subscription<zed_msgs::msg::Cones>(
            "/zed2i/topic_bbox_zed3d", 1, std::bind(&LoadDataNode::timer_callback, this, _1));

        // Publishers
        publisher_waypoints_ = this->create_publisher<control_msgs::msg::WaypointArrayStamped>("Waypoints", 1);
      
        publisher_ref_waypoint_ = this->create_publisher<control_msgs::msg::WaypointArrayStamped>("Waypoint_filtered", 1);
        publisher_spline_points_ = this->create_publisher<control_msgs::msg::WaypointArrayStamped>("Spline_Points", 1);
        publisher_special_case_ = this->create_publisher<path_interfaces::msg::Pubxy>("Special_case", 1);
        publisher_velocity_ = this->create_publisher<control_msgs::msg::RefData>("ReferenceData", 1);
        publisher_filtered_cones_ = this->create_publisher<zed_msgs::msg::Cones>("Filtered_Cones", 1);
    }

private:
    void timer_callback(zed_msgs::msg::Cones msg_cones)
    {
        std::vector<double> xBlue, yBlue, xYellow, yYellow, xBigO, yBigO, xLittleO, yLittleO;
        zed_msgs::msg::Cones filtered_cones;

        // Esegui lo spacchettamento e filtraggio dei coni
        for (const auto &cone : msg_cones.big_orange_cones)
        {
            xBigO.push_back(cone.x);
            yBigO.push_back(cone.y);
        }

        for (const auto &cone : msg_cones.little_orange_cones)
        {
            xLittleO.push_back(cone.x);
            yLittleO.push_back(cone.y);
            if (cone.y > 0)
            {
                xBlue.push_back(cone.x);
                yBlue.push_back(cone.y);
            }
            else
            {
                xYellow.push_back(cone.x);
                yYellow.push_back(cone.y);
            }
        }

        for (const auto &cone : msg_cones.blue_cones)
        {
            if (std::abs(cone.y) < filter_param_y_ && std::sqrt(cone.x * cone.x + cone.y * cone.y) < filter_param_distance_)
            {
                xBlue.push_back(cone.x);
                yBlue.push_back(cone.y);
            }
        }

        for (const auto &cone : msg_cones.yellow_cones)
        {
            if (std::abs(cone.y) < filter_param_y_ && std::sqrt(cone.x * cone.x + cone.y * cone.y) < filter_param_distance_)
            {
                xYellow.push_back(cone.x);
                yYellow.push_back(cone.y);
            }
        }

        // Pubblica i coni filtrati
        for (size_t i = 0; i < xLittleO.size(); ++i)
        {
            zed_msgs::msg::Cone little_orange_cone;
            little_orange_cone.cone_type = 3;
            little_orange_cone.x = xLittleO[i];
            little_orange_cone.y = yLittleO[i];
            little_orange_cone.z = -1.3;
            filtered_cones.little_orange_cones.push_back(little_orange_cone);
        }

        for (size_t i = 0; i < xBigO.size(); ++i)
        {
            zed_msgs::msg::Cone big_orange_cone;
            big_orange_cone.cone_type = 1;
            big_orange_cone.x = xBigO[i];
            big_orange_cone.y = yBigO[i];
            big_orange_cone.z = -1.3;
            filtered_cones.big_orange_cones.push_back(big_orange_cone);
        }

        for (size_t i = 0; i < xBlue.size(); ++i)
        {
            zed_msgs::msg::Cone blue_cone;
            blue_cone.cone_type = 2;
            blue_cone.x = xBlue[i];
            blue_cone.y = yBlue[i];
            blue_cone.z = -1.3;
            filtered_cones.blue_cones.push_back(blue_cone);
        }

        for (size_t i = 0; i < xYellow.size(); ++i)
        {
            zed_msgs::msg::Cone yellow_cone;
            yellow_cone.cone_type = 4;
            yellow_cone.x = xYellow[i];
            yellow_cone.y = yYellow[i];
            yellow_cone.z = -1.3;
            filtered_cones.yellow_cones.push_back(yellow_cone);
        }
        
        //caso base delaunay ovvero almeno tre coni di diverso colore
        if((xBlue.size() >0 && xYellow.size() >0) && (xBlue.size() + xYellow.size() > 2))
        {
            if((xBlue.size()>1 && xYellow.size()>0) || (xBlue.size()>0 && xYellow.size()>1))
            {
                Waypoints = delaunayCalculation();
            }
        }

        //richiamo funzione spline
        spline(max_spline_degree_, Waypoints);

        // Pubblica i coni filtrati
        RCLCPP_INFO(this->get_logger(), "--- Pubblico i coni filtrati ---");
        publisher_filtered_cones_->publish(filtered_cones);
    }

    

    std::vector<CustomPoint2D> delaunayCalculation() {

        std::vector<CustomPoint2D> points;
        // Itera su ogni "cone" in filtered_cones (di tipo zed_msgs::msg::Cones) per convertire filtered_cones in custompoint2D
        for (const auto& cone : filtered_cones.blue_cones) {  
            CustomPoint2D point;
            point.data[0] = cone.x;  
            point.data[1] = cone.y;  

            points.push_back(point);
        }
        for (const auto& cone : filtered_cones.yellow_cones) {  
            CustomPoint2D point;
            point.data[0] = cone.x;  
            point.data[1] = cone.y;  

            points.push_back(point);
        }
        for (const auto& cone : filtered_cones.little_orange_cones) {  
            CustomPoint2D point;
            point.data[0] = cone.x;  
            point.data[1] = cone.y;  

            points.push_back(point);
        }
        for (const auto& cone : filtered_cones.big_orange_cones) {  
            CustomPoint2D point;
            point.data[0] = cone.x;  
            point.data[1] = cone.y;  

            points.push_back(point);
        }


        std::vector<CustomEdge> edges;
        CDT::Triangulation<double> cdt; 
        cdt.eraseSuperTriangle();

        //triangolo
        cdt.insertVertices(

            points.begin(),    points.end(),    [](const CustomPoint2D& p){ return p.data[0]; },

            [](const CustomPoint2D& p){ return p.data[1]; }
        );
        //calcolo waypoints (punto centrale edges)
        CDT::EdgeUSet lati = CDT::extractEdgesFromTriangles(cdt.triangles);
            bool foundBV1 = false;
            bool foundBV2 = false;
            //controllo che gli edgedes stanno nel vettore xblue o xyellow 
            if((!xBlue.empty() && !yBlue.empty()))
            {
                for(auto lato : lati){
                    for(size_t i = 0; i < xBlue.size(); ++i)
                    {
                        if(cdt.vertices[lato.v1()].x == xBlue[i])
                        {
                            for(size_t j = 0; j < yBlue.size(); ++j)
                            {
                                if(cdt.vertices[lato.v1()].y == yBlue[j])
                                {
                                    bool foundBV1 = true;
                                }
                            }
                        }
                        lato.v1();

                    }
                    for(size_t i = 0; i < xBlue.size(); ++i)
                    {
                   
                        if(cdt.vertices[lato.v2()].x == xBlue[i])
                        {
                            for(size_t i = 0; i < yBlue.size(); ++i)
                            {
                                if(cdt.vertices[lato.v2()].y == yBlue[i])
                                {
                                    bool foundBV2 = true;
                                }
                            }
                        }
                        lato.v2();
                    }
                    if(foundBV1 ^ foundBV2) //XOR 
                    {
                        double midX = (cdt.vertices[lato.v1()].x  + cdt.vertices[lato.v2()].x) / 2.0;     
                        double midY = (cdt.vertices[lato.v1()].y + cdt.vertices[lato.v2()].y) / 2.0;
                        Waypoints.push_back({midX, midY});
                       
                    }
                }
            }

            
        return Waypoints;
        
    }
    std::vector<glm::vec2> spline(const int max_spline_degree_, const  std::vector<CustomPoint2D>Waypoints) {
    // DEBUG ONLY grado
    if (max_spline_degree_ != 2)
        throw std::invalid_argument("Grado non valido");

    // Estrai coordinate x e y
    std::vector<double> x, y;
    for (const auto& p : Waypoints) {
        x.push_back(p.data[0]);
        y.push_back(p.data[1]);
    }

    // Crea vettore t (intervallo normalizzato)
    std::vector<double> t(Waypoints.size());
    for (size_t i = 0; i < t.size(); ++i) {
        t[i] = static_cast<double>(i) / (t.size() - 1);
    }

    // Calcolo spline quadratiche
    std::vector<double> spline_x, spline_y;
    spapi(max_spline_degree_, x, spline_x);
    spapi(max_spline_degree_, y, spline_y);

    // Punto finale spline
    double x_end = fnval(spline_x, 1.0);
    double y_end = fnval(spline_y, 1.0);
    glm::vec2 punto_fine(x_end, y_end);
/*
    // Tangente nell'ultimo punto
    double dx = fnval(spline_x, 1.0) - fnval(spline_x, 0.9);
    double dy = fnval(spline_y, 1.0) - fnval(spline_y, 0.9);
    glm::vec2 vettore_tangente(dx, dy);
    vettore_tangente = glm::normalize(vettore_tangente);

    // Calcola distanze
    std::vector<double> distanze(adiacenti_correnti.size());
    for (size_t i = 0; i < adiacenti_correnti.size(); ++i) {
        glm::vec2 cono = adiacenti_correnti[i];
        glm::vec2 vettore_cono = cono - punto_fine;
        double distanza_perpendicolare = std::abs(glm::determinant(glm::dmat2(vettore_tangente, glm::dvec2(vettore_cono)))) / glm::length(vettore_tangente);
        distanze[i] = distanza_perpendicolare;
    }

    // Trova il minimo
    auto min_it = std::min_element(distanze.begin(), distanze.end());
    int indice_minimo = std::distance(distanze.begin(), min_it);
    glm::vec2 cono_scelto = adiacenti_correnti[indice_minimo];
*/
    // Aggiorna punti
    std::vector<glm::vec2> punti_aggiornati ;
    for (const auto& p : Waypoints) {  
    glm::vec2 vec(p.data[0], p.data[1]);  // Converte CustomPoint2D in glm::vec2
    punti_aggiornati.push_back(vec);  
    }

    return punti_aggiornati;
}

void spapi(int grado, const std::vector<double>& valori, std::vector<double>& spline)
{
    // DEBUG ONLY grado
    if (grado != 2)
        throw std::invalid_argument("Grado non valido");

    // Interpolazione per spline quadratica (grado = 2)
    int n = valori.size();
    spline.resize(n);

    for (int i = 1; i < n - 1; ++i) {
        spline[i] = (valori[i - 1] + valori[i] + valori[i + 1]) / 3.0;
    }

    // Estremi non hanno punti sufficienti per media, li manteniamo uguali
    spline[0] = valori[0];
    spline[n - 1] = valori[n - 1];
}
double fnval(const std::vector<double>& spline, double t)
{
    // Valutazione spline con interpolazione lineare
    int n = spline.size();
    if (t <= 0) return spline[0];
    if (t >= 1) return spline[n - 1];
    int idx = static_cast<int>(t * (n - 1));
    double alpha = t * (n - 1) - idx;
    return (1 - alpha) * spline[idx] + alpha * spline[idx + 1];
}


    /*--------------------PUBLISHERS--------------------*/

    //funzione publisher spline
    void publish_spline_points(const std::vector<std::vector<float>>& punti_aggiornati)
    {
        // Create the message
        auto msg = control_msgs::msg::WaypointArrayStamped();
        
        // Loop over the spline points and fill the message
        for (size_t i = 0; i < punti_aggiornati.size(); ++i)
        {
            auto spline_point = control_msgs::msg::Waypoint();
            spline_point.position.x = punti_aggiornati[i][0];
            spline_point.position.y = punti_aggiornati[i][1];
            msg.waypoints.push_back(spline_point);
        }
        
        // Publish the message
        publisher_spline_points_->publish(msg);
    }

    void publish_waypoints(const std::vector<CustomPoint2D> Waypoints)
    {
        auto msg = control_msgs::msg::WaypointArrayStamped();

        for(size_t i = 0; i < Waypoints.size(); ++i)
        {
            auto waypoint = control_msgs::msg::Waypoint();
            waypoint.position.x = Waypoints[i].data[0];
            waypoint.position.y = Waypoints[i].data[1];
            msg.waypoints.push_back(waypoint); 
        }

        publisher_waypoints_->publish(msg);
    }

    // Member variables
    double leng_;
    int search_region_;
    int eval_length_;
    int numWaypoints_to_spline_;
    int spline_curvature_constrain_;
    double Vmax_;
    double Vmin_;
    int RCmax_;
    double filter_param_distance_;
    double filter_param_y_;
    int max_spline_degree_;
    int waypoint_threshold_;
    int special_case_;
    int spline_degree_;
    bool SpecialActivated_;
    bool la_dyn_;
    int pred_eval_length_;
    double avg_curvature_;
    bool arclen_minore_;
    std::vector<double> xBlue, yBlue, xYellow, yYellow, xBigO, yBigO, xLittleO, yLittleO;
    std::vector<CustomPoint2D> Waypoints;
    zed_msgs::msg::Cones filtered_cones;
    std::vector<glm::vec2> punti_aggiornati;

    // Subscribers and Publishers
    rclcpp::Subscription<zed_msgs::msg::Cones>::SharedPtr subscription_cones_;
    rclcpp::Publisher<control_msgs::msg::WaypointArrayStamped>::SharedPtr publisher_waypoints_;
    rclcpp::Publisher<path_interfaces::msg::WaypointsPlusCones>::SharedPtr publisher_wayspluscones_;
    rclcpp::Publisher<control_msgs::msg::WaypointArrayStamped>::SharedPtr publisher_ref_waypoint_;
    rclcpp::Publisher<control_msgs::msg::WaypointArrayStamped>::SharedPtr publisher_spline_points_;
    rclcpp::Publisher<path_interfaces::msg::Pubxy>::SharedPtr publisher_special_case_;
    rclcpp::Publisher<control_msgs::msg::RefData>::SharedPtr publisher_velocity_;
    rclcpp::Publisher<zed_msgs::msg::Cones>::SharedPtr publisher_filtered_cones_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LoadDataNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


/*
#include "CDT.h"


struct CustomPoint2D{

    double data[2];

};

struct CustomEdge{

    std::pair<std::size_t, std::size_t> vertices;

};
class LoadDataNode : public rclcpp::Node {
public: 
     LoadDataNode() : Node("load_data_node") {
        RCLCPP_INFO(this->get_logger(), "Node has started");

        // containers other than std::vector will work toostd::vector<CustomPoint2D> points = ; 

        std::vector<CustomEdge> edges;
        std::vector<CustomPoint2D> points = {{1, 11}, {5, 5}, {5, 8}, {4, 10}, {-2, 11}, {-3, 10}, {-4, 8},
                                            {8, 5}, {8, 8}, {6.5, 10.75}, {4.75, 13.5}, {1.64, 14.7},
                                            {-2, 14}, {-4, 12.3}, {-5.32, 10.17}};
        CDT::Triangulation<double> cdt;

        cdt.insertVertices(

            points.begin(),    points.end(),    [](const CustomPoint2D& p){ return p.data[0]; },

            [](const CustomPoint2D& p){ return p.data[1]; }

        );
        
        cdt.insertEdges(

            edges.begin(),    edges.end(),    [](const CustomEdge& e){ return e.vertices.first; },

            [](const CustomEdge& e){ return e.vertices.second; }

        );
        
     }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LoadDataNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
*/ 
