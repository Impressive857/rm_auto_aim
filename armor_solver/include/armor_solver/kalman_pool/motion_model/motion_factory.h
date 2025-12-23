//
// Created by lbw on 25-4-15.
//

#ifndef MOTION_FACTORY_H
#define MOTION_FACTORY_H
#include <string>
#include <mutex>
#include <unordered_map>
#include <functional>

#include <armor_solver/kalman_pool/motion_model/moving_top.h>
#include <armor_solver/kalman_pool/motion_model/fix_top.h>
#include <armor_solver/kalman_pool/motion_model/translate.h>


template <typename ProductBase>
class ModuleFactory
{
public:
  struct ModuleMeta
  {
    std::string module_name;
    std::string file_path;
    int line_number;
    time_t register_time;
  };

  static ModuleFactory& instance()
  {
    if (_instance == nullptr)
    {
      _instance = new ModuleFactory();
    }
    return *_instance;
  }

  ~ModuleFactory() { delete _instance; }

  bool registerModule(const std::string& module_name, std::function<ProductBase*()> creator,
                      const std::string& file, int line)
  {
    std::lock_guard<std::mutex> lock(_mutex);
    if (module_creators_.contains(module_name))
    {
      std::cerr << "[ERROR] Module " << module_name
        << " already registered at "
        << module_metas_[module_name].file_path << ":"
        << module_metas_[module_name].line_number << "\n";
      return false;
    }

    module_creators_[module_name] = creator;
    module_creators_[module_name] = {file, line, time(nullptr)};

    return true;
  }

  ProductBase* createModule(const std::string& module_name)
  {
    auto it = module_creators_.find(module_name);
    if (it != module_creators_.end())
    {
      try
      {
        return it->second();
      }
      catch (const std::exception& e)
      {
        std::cerr << "[ERROR] Module " << module_name
          << " failed:" << e.what() << "\n";
      }
    }
    return nullptr;
  }

  const ModuleMeta& getModuleMeta(const std::string& module_name)
  {
    auto it = module_metas_.find(module_name);
    if (it != module_metas_.end())
    {
      try
      {
        return it->second();
      }
      catch (const std::exception& e)
      {
        std::cerr << "[ERROR] Module " << module_name
          << " failed:" << e.what() << "\n";
      }
    }
    return nullptr;
  }

  std::vector<std::string> getModuleNames()
  {
    std::lock_guard<std::mutex> lock(_mutex);
    std::vector<std::string> names;
    for (auto pair : module_creators_)
    {
      names.push_back(pair.first);
    }
    return names;
  }

private:
  ModuleFactory() = default;
  ModuleFactory(const ModuleFactory&) = delete;
  ModuleFactory& operator=(const ModuleFactory&) = delete;
  static ModuleFactory* _instance;

  std::mutex _mutex;
  std::unordered_map<std::string, ModuleMeta> module_metas_;
  std::unordered_map<std::string, ProductBase*> module_creators_;

  void printModuleTrace(const std::string& module_name)
  {
    const auto& meta = module_metas_[module_name];
    std::cout << "Module Trace:\n"
      << "  Registered at: " << meta.file_path << ":" << meta.line_number << "\n"
      << "  Timestamp: " << std::put_time(std::localtime(&meta.register_time), "%F %T") << "\n";
  }
};


#endif //MOTION_FACTORY_H
