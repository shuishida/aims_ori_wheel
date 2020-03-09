#ifndef TOPMAP_MANAGER_H
#define TOPMAP_MANAGER_H

#include "node_controller.h"
#include "ros/ros.h"

#include <stdio.h>
#include <sstream>

#include <QList>
#include <QObject>
#include <QStringList>

#include "rviz/display_context.h"
#include "rviz/pluginlib_factory.h"
#include "rviz/properties/enum_property.h"
#include "rviz/properties/property.h"
#include "rviz/properties/property_tree_model.h"
#include "rviz/render_panel.h"

namespace topological_rviz_tools
{

class TopmapManager: public QObject
{
Q_OBJECT
public:
  TopmapManager(rviz::DisplayContext* context);
  ~TopmapManager();

  void initialize();

  void update(float wall_dt, float ros_dt);

  /** @brief Return the current NodeController in use for the main
   * RenderWindow. */
  NodeController* getController() const;

  NodeProperty* getCurrent() const;

  NodeController* create(const QString& type);

  int getNumViews() const;

  NodeController* getViewAt(int index) const;

  /** @brief Remove the given NodeController from the list and return
   * it.  If it is not in the list, NULL is returned and nothing
   * changes. */
  NodeController* take(NodeController* view);

  /** @brief Remove the NodeController at the given index from the
   * list and return it.  If the index is not valid, NULL is returned
   * and nothing changes. */
  NodeController* takeAt(int index);

  rviz::PropertyTreeModel* getPropertyModel() { return property_model_; }

  void load(const rviz::Config& config);
  void save(rviz::Config config) const;

  /** @brief Make a copy of @a view_to_copy and install that as the new current NodeController. */
  void setCurrentFrom(NodeProperty* view_to_copy);

  /** @brief Return a copy of source, made by saving source to
   * a Config and instantiating and loading a new one from that. */
  NodeProperty* copy(NodeProperty* source);

  rviz::PluginlibFactory<NodeController>* getFactory() const { return factory_; }

  /** @brief Set the 3D view widget whose view will be controlled by
   * NodeController instances from by this TopmapManager. */
  void setRenderPanel(rviz::RenderPanel* render_panel);

  /** @brief Return the 3D view widget managed by this TopmapManager. */
  rviz::RenderPanel* getRenderPanel() const { return render_panel_; }

public Q_SLOTS:

  /** @brief Make a copy of the current NodeController and add it to the end of the list of saved views. */
  void copyCurrentToList();

  /** @brief Create a new view controller of the given type and set it
   * up to mimic and replace the previous current view. */
  void setCurrentNodeControllerType(const QString& new_class_id);

Q_SIGNALS:
  void configChanged();

  /** @brief Emitted just after the current view controller changes. */
  void currentChanged();

private Q_SLOTS:
  void onCurrentDestroyed(QObject* obj);

private:
  /** @brief Set @a new_current as current.
   * @param mimic_view If true, call new_current->mimic(previous), if false call new_current->transitionFrom(previous).
   *
   * This calls mimic() or transitionFrom() on the new controller,
   * deletes the previous controller (if one existed), and tells the
   * RenderPanel about the new controller. */
  void setCurrent(NodeProperty* new_current, bool mimic_view);

  rviz::DisplayContext* context_;
  NodeController* root_property_;
  rviz::PropertyTreeModel* property_model_;
  rviz::PluginlibFactory<NodeController>* factory_;
  NodeProperty* current_;
  rviz::RenderPanel* render_panel_;
};

}
#endif
