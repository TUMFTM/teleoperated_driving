/**
 * @file model_loader.hpp
 * @brief Loads the vehicles model e.g. TUM EDGAR into the 3D Scene
 * @copyright 2024 TUMFTM
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "tod_gl/scene/entity.hpp"
#include "tod_gl/renderer/data_container.hpp"

#include <assimp/scene.h>
#include <boost/filesystem.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>

namespace tod_gl {

class ModelLoader {
  public:
    ModelLoader(std::shared_ptr<Scene> activeScene, const std::string &modelPath);
    Entity load_model(const std::string &modelName, Entity parent, const glm::vec3 &scaling, const glm::vec3 &rotation,
                     const std::string &packagePath);

  private:
    std::vector<Texture> _textures;
    std::vector<std::string> _texturePaths;
    std::vector<Mesh> _meshes;
    std::shared_ptr<Scene> _active_scene;
    std::string _modelPath;
    std::string _packagePath;
    std::string _modelName;
    Texture _emptyTexture;
    Texture texture_from_file(const char *path, const int shader);
    void process_node(aiNode *node, const aiScene *scene, const int shader);
    Mesh process_mesh(aiMesh *mesh, const aiScene *scene, const int shader);
    std::vector<Texture> load_material_textures(aiMaterial *mat, aiTextureType type, const int shader);
    /* Returns the first found file with given extension under the given root directory */
    boost::filesystem::path get_first_with_extention(const boost::filesystem::path &root, const std::string &extention);
};

};  // namespace tod_gl
