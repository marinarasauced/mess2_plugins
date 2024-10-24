
#include "mess2_plugins/threat.hpp"

namespace mess2_plugins
{
    std::tuple<arma::mat, arma::mat> get_meshgrid(const arma::vec& x_, const arma::vec& y_)
    {
        int num_x = x_.n_elem;
        int num_y = y_.n_elem;
        arma::mat x_mesh(num_y, num_x);
        arma::mat y_mesh(num_y, num_x);
        for (int iter = 0; iter < num_y; ++iter)
        {
            x_mesh.row(iter) = x_.t();
            y_mesh.row(iter).fill(y_(iter));
        }
        return std::make_tuple(x_mesh, y_mesh);
    }

    arma::mat generate_threat(const arma::mat& x_mesh, const arma::mat& y_mesh)
    {
        arma::mat threat = arma::mat(x_mesh.n_rows, y_mesh.n_cols, arma::fill::zeros);

        int16_t n_peaks = 11;
        arma::mat coeff_peaks = {
            { 0.4218, 1.4253, 0.8271,  1.5330,  1.7610,  0.4533,  0.2392,  0.8364,  0.5060, 1.5190,  2},
            {-4.7996, 6.8744, 1.6560,  4.3881, -3.1295, -9.8145, -6.1511,  0.1478, -9.5152, 1.0008,  9},
            {-4.4763, 3.9248, 7.6271, -9.5064, -3.1759, -1.5719, -8.3998, -8.4129, -8.5525, 8.0068, -1},
            { 3.2579, 1.5239, 1.2908,  2.0099,  2.7261,  2.7449,  2.9398,  2.7439,  0.3691, 3.1097,  3},
            { 0.4039, 0.4382, 2.4844,  1.9652,  1.9238,  1.8567,  0.5470,  1.0401,  0.7011, 3.3193,  3}
        };
        double c1 = 5.0;

        for (int iter = 0; iter < n_peaks; ++iter)
        {
            arma::mat c_xym = exp(
                -pow((x_mesh - coeff_peaks(1, iter)), 2) / (2 * pow(coeff_peaks(3, iter), 2))
                -pow((y_mesh - coeff_peaks(2, iter)), 2) / (2 * pow(coeff_peaks(4, iter), 2))
            );
            threat += coeff_peaks(0, iter) * c_xym;
        }
        threat = c1 * threat + 1;
        return threat;
    }

    mess2_msgs::msg::VertexArray get_vertices(const arma::mat& threat, const arma::mat& x_mesh, const arma::mat& y_mesh)
    {
        int n_rows = threat.n_rows;
        int n_cols = threat.n_cols;
        mess2_msgs::msg::VertexArray vertices;
        for (int iter = 0; iter < n_rows; ++iter)
        {
            for (int jter = 0; jter < n_cols; ++jter)
            {
                mess2_msgs::msg::Vertex vertex;
                vertex.position.x = x_mesh(iter, jter);
                vertex.position.y = y_mesh(iter, jter);
                vertex.threat = threat(iter, jter);
                vertices.vertices.push_back(vertex);
            }
        }
        return vertices;
    }

    mess2_msgs::msg::EdgeArray get_edges(const arma::mat& threat)
    {
        int n_rows = threat.n_rows;
        int n_cols = threat.n_cols;
        mess2_msgs::msg::EdgeArray edges;
        for (int iter = 0; iter < n_rows; ++iter)
        {
            for (int jter = 0; jter < n_cols; ++jter)
            {
                int vertex_curr = iter * n_cols + jter;
                if (jter < n_cols - 1)
                {
                    int vertex_adj = iter * n_cols + (jter + 1);
                    mess2_msgs::msg::Edge edge;
                    edge.index1 = vertex_curr;
                    edge.index2 = vertex_adj;
                    edges.edges.push_back(edge);
                }
                if (iter < n_rows - 1)
                {
                    int vertex_adj = (iter + 1) * n_cols + jter;
                    mess2_msgs::msg::Edge edge;
                    edge.index1 = vertex_curr;
                    edge.index2 = vertex_adj;
                    edges.edges.push_back(edge);
                }
                if (iter < n_rows - 1 && jter < n_cols - 1)
                {
                    int vertex_adj = (iter + 1) * n_cols + (jter + 1);
                    mess2_msgs::msg::Edge edge;
                    edge.index1 = vertex_curr;
                    edge.index2 = vertex_adj;
                    edges.edges.push_back(edge);   
                }
                if (iter < n_rows - 1 && jter > 0)
                {
                    int vertex_adj = (iter + 1) * n_cols + (jter - 1);
                    mess2_msgs::msg::Edge edge;
                    edge.index1 = vertex_curr;
                    edge.index2 = vertex_adj;
                    edges.edges.push_back(edge);   
                }
            }
        }
        return edges; 
    }

    mess2_msgs::msg::ThreatField get_threat_field(const int resolution)
    {
        arma::vec x_ = arma::linspace(-15.0, 15.0, resolution);
        arma::vec y_ = arma::linspace(-15.0, 15.0, resolution);
        arma::vec x_scaled = arma::linspace(-3.0, 3.0, resolution);
        arma::vec y_scaled = arma::linspace(-3.0, 3.0, resolution);
        auto [x_mesh, y_mesh] = get_meshgrid(x_, y_);
        auto threat = generate_threat(x_mesh, y_mesh);
        auto vertices = get_vertices(threat, x_mesh, y_mesh);
        auto edges = get_edges(threat);

        mess2_msgs::msg::ThreatField threat_field;
        threat_field.vertices = vertices;
        threat_field.edges = edges;
        return threat_field;
    }

    int64_t get_vertex_index(const mess2_msgs::msg::VertexArray vertices, geometry_msgs::msg::Point::SharedPtr point)
    {
        int64_t index = -1;
        double min_distance = std::numeric_limits<double>::max();
        double x = point->x;
        double y = point->y;
        double z = point->z;
        for (std::vector<mess2_msgs::msg::Vertex>::size_type iter = 0; iter < vertices.vertices.size(); ++iter)
        {
            const auto &position = vertices.vertices[iter].position;
            double distance = std::sqrt(
                std::pow(position.x - x, 2) +
                std::pow(position.y - y, 2) + 
                std::pow(position.z - z, 2)
            );
            if (distance < min_distance)
            {
                min_distance = distance;
                index = static_cast<int64_t>(iter);
            }
        }
        return index;
    }

    std::vector<std::array<double, 3>> get_colormap(const std::string &file_path)
    {
        std::ifstream file(file_path);
        if (!file.is_open())
        {
            return {};
        }

        std::vector<std::array<double, 3>> colormap;
        std::string line;
        while (std::getline(file, line))
        {
            std::istringstream ss(line);
            std::string item;
            std::array<double, 3> color;
            std::getline(ss, item, ',');
            color[0] = std::stof(item);
            std::getline(ss, item, ',');
            color[1] = std::stof(item);
            std::getline(ss, item, ',');
            color[2] = std::stof(item);
            colormap.push_back(color);
        }
        return colormap;
    }

    sensor_msgs::msg::Image get_threat_field_image(const arma::mat& threat, const std::vector<std::array<double, 3>> colormap)
    {
        sensor_msgs::msg::Image image;
        image.header.frame_id = "map";

        int32_t width = threat.n_cols;
        int32_t height = threat.n_rows;
        image.width = width;
        image.height = height;
        image.encoding = "rgb8";
        image.step = width * 3;
        image.data.resize(width * height * 3);

        double threat_min = threat.min();
        double threat_max = threat.max();
        for (int32_t iter = 0; iter < height; ++iter)
        {
            for (int32_t jter = 0; jter < width; ++jter)
            {
                double threat_val = threat(iter, jter);
                double norm_threat_val = (threat_val - threat_min) / (threat_max - threat_min);
                norm_threat_val = std::max(0.0, std::min(1.0, norm_threat_val));

                uint8_t r, g, b;
                int32_t index_colormap = static_cast<int32_t>(norm_threat_val * (colormap.size() - 1));
                index_colormap = std::max(0, std::min(index_colormap, static_cast<int32_t>(colormap.size()) - 1));

                const auto &color = colormap[index_colormap];
                r = color[2] * 255;
                g = color[1] * 255;
                b = color[0] * 255;

                int64_t index_image = (iter * width + jter) * 3;
                image.data[index_image + 0] = r;
                image.data[index_image + 1] = g;
                image.data[index_image + 2] = b;
            }
        }
        return image;
    }


    mess2_msgs::msg::ThreatElementArray get_threat_field_image_vertices(const mess2_msgs::msg::VertexArray vertices, const double r, const double g, const double b)
    {
        mess2_msgs::msg::ThreatElementArray elements;
        for (size_t iter = 0; iter < vertices.vertices.size(); ++iter)
        {
            mess2_msgs::msg::ThreatElement element;
            element.index = iter;
            element.color.r = r;
            element.color.g = g;
            element.color.b = b;
            element.color.a = 1.0;
            elements.elements.push_back(element);
        }
        return elements;
    }

    mess2_msgs::msg::ThreatElementArray get_threat_field_image_edges(const mess2_msgs::msg::EdgeArray edges, const double r, const double g, const double b)
    {
        mess2_msgs::msg::ThreatElementArray elements;
        for (size_t iter = 0; iter < edges.edges.size(); ++ iter)
        {
            mess2_msgs::msg::ThreatElement element;
            element.index = iter;
            element.color.r = r;
            element.color.g = g;
            element.color.b = b;
            element.color.a = 1.0;
            elements.elements.push_back(element);
        }
        return elements;
    }

    class Actor
    {
    public:
        Actor(const std::string& actor_name, const std::string& actors_path) : actor_name(actor_name)
        {
            std::string actor_config_path = actors_path + "/" + actor_name + "/config.yaml";


        }

        void load_config(const std::string& config_path)
        {

        }

        std::string get_actor_name()
        {
            return actor_name;
        }

        std::string get_turtlebot3_model()
        {
            return turtlebot3_model;
        }

    private:
        std::string actor_name;
        std::string turtlebot3_model;
        std::string lds_model;
        double k_lin;
        double k_ang;
        double speed;
        double radius;
        double u_lin_max;
        double u_ang_max;
    };
}