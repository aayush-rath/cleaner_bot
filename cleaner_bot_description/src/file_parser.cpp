#include "file_parser.h"

std::string get_mesh_path(const std::string& model_sdf_path, const std::string& model_root, const std::string& model_name) {
    tinyxml2::XMLDocument model_sdf;
    if (model_sdf.LoadFile(model_sdf_path.c_str()) != tinyxml2::XML_SUCCESS) {
        std::cerr << "[ERROR] Could not load SDF: " << model_sdf_path << std::endl;
        return "";
    }

    tinyxml2::XMLElement* mesh_uri = model_sdf.FirstChildElement("sdf")->FirstChildElement("model")->FirstChildElement("link");

    while (mesh_uri) {
        auto* visual = mesh_uri->FirstChildElement("visual");
        if (visual) {
            auto* geometry = visual->FirstChildElement("geometry");
            if (geometry) {
                auto* mesh = geometry->FirstChildElement("mesh");
                if (mesh) {
                    auto* uri = mesh->FirstChildElement("uri");
                    if (uri && uri->GetText()) {
                        std::string uri_text = uri->GetText();
                        const std::string prefix = "model://";
                        if (uri_text.rfind(prefix, 0) == 0) {
                            std::string relative_path = uri_text.substr(prefix.length());
                            return model_root + '/' + relative_path;
                        } else {
                            return model_root+'/'+uri_text;
                        }
                    }
                }
            }
        }
        mesh_uri = mesh_uri->NextSiblingElement("link");
    }

    std::cerr << "[WARN] No <mesh><uri> tags found in model.sdf of " << model_name << std::endl;
    return "";
}

void read_world_file(const std::string& world_file_path, std::unordered_map<std::string, std::vector<double>>& models) {
    tinyxml2::XMLDocument world_file;
    if (world_file.LoadFile(world_file_path.c_str()) != tinyxml2::XML_SUCCESS) {
        std::cerr << "[ERROR] Could not load world file: " << world_file_path << std::endl;
        return;
    }

    tinyxml2::XMLElement* sdf_include = world_file.FirstChildElement("sdf")->FirstChildElement("world")->FirstChildElement("include");

    while (sdf_include) {
        auto* uri = sdf_include->FirstChildElement("uri");
        if (uri && uri->GetText()) {
            std::string uri_text = uri->GetText();
            const std::string prefix = "model://";

            if (uri_text.rfind(prefix, 0) == 0) {
                std::string model_name = uri_text.substr(prefix.length());

                std::vector<double> pose(6, 0.0);
                auto* pose_element = sdf_include->FirstChildElement("pose");
                if (pose_element && pose_element->GetText()) {
                    std::stringstream ss(pose_element->GetText());
                    for (int i = 0; i < 6; ++i) ss >> pose[i];
                }

                models[model_name] = pose;
            }
        }
        sdf_include = sdf_include->NextSiblingElement("include");
    }
}