#include "agent_profile_table.h"

//===================================================
AgentProfileTab::AgentProfileTab(CrowdSimImplPtr crowd_sim_impl)
: TableList(13), implPtr(crowd_sim_impl)
{
  const QStringList labels =
  { "Name", "class", "max_accel", "max_angle_vel", "max_neighbors",
    "max_speed", "neighbor_dist", "obstacle_set", "pref_speed", "r",
    "ORCA_tau", "ORCA_tauObst", ""};
  label_size = labels.size();

  setHorizontalHeaderLabels(labels);
  setMinimumSize(1600, 400);
  setSizePolicy(
    QSizePolicy::Expanding,
    QSizePolicy::MinimumExpanding);
}

//===================================================
void AgentProfileTab::add_button_clicked()
{
  save();
  implPtr->agent_profiles.emplace_back("new profile");
}

//===================================================
void AgentProfileTab::list_agent_profile_in_impl()
{

  auto profile_count = implPtr->agent_profiles.size();

  for (size_t i = 0; i < profile_count; i++)
  {
    auto& current_profile = implPtr->agent_profiles[i];
    setItem(i, 0,
      new QTableWidgetItem(QString::fromStdString(current_profile.profile_name)));
    setItem(i, 1,
      new QTableWidgetItem(QString::number(static_cast<uint>(current_profile.
      profile_class))));
    setItem(i, 2,
      new QTableWidgetItem(QString::number(current_profile.max_accel)));
    setItem(i, 3,
      new QTableWidgetItem(QString::number(current_profile.max_angle_vel)));
    setItem(i, 4,
      new QTableWidgetItem(QString::number(static_cast<uint>(current_profile.
      max_neighbors))));
    setItem(i, 5,
      new QTableWidgetItem(QString::number(current_profile.max_speed)));
    setItem(i, 6,
      new QTableWidgetItem(QString::number(current_profile.neighbor_dist)));
    setItem(i, 7,
      new QTableWidgetItem(QString::number(static_cast<uint>(current_profile.
      obstacle_set))));
    setItem(i, 8,
      new QTableWidgetItem(QString::number(current_profile.pref_speed)));
    setItem(i, 9, new QTableWidgetItem(QString::number(current_profile.r)));
    setItem(i, 10,
      new QTableWidgetItem(QString::number(current_profile.ORCA_tau)));
    setItem(i, 11,
      new QTableWidgetItem(QString::number(current_profile.ORCA_tauObst)));

    // not allowed to delete the external agent profile
    if (i == 0)
      continue;
    QPushButton* delete_button = new QPushButton("Delete", this);
    setCellWidget(i, 12, delete_button);
    connect(
      delete_button,
      &QAbstractButton::clicked,
      [&, i]()
      {
        implPtr->agent_profiles.erase(implPtr->agent_profiles.begin() + i);
        update();
      }
    );
  }
}

//===================================================
void AgentProfileTab::update()
{
  blockSignals(true);
  auto profiles_number = implPtr->agent_profiles.size();
  setRowCount(1 + profiles_number);
  clearContents();
  list_agent_profile_in_impl();

  QPushButton* add_button = new QPushButton("Add...", this);
  for (auto i = 0; i < label_size - 1; i++)
  {
    setItem(profiles_number, i,
      new QTableWidgetItem(QString::fromStdString("")));
  }
  setCellWidget(profiles_number, label_size - 1, add_button);
  connect(
    add_button,
    &QAbstractButton::clicked,
    [&]()
    {
      add_button_clicked();
      update();
    }
  );
  blockSignals(false);
}

//===================================================
int AgentProfileTab::save()
{
  auto row_count = rowCount();

  for (auto i = 1; i < row_count - 1; i++)
  {
    auto& current_profile = implPtr->agent_profiles.at(i);

    QTableWidgetItem* pItem = item(i, 0);
    auto profile_name = pItem->text().toStdString();
    current_profile.profile_name = profile_name;

    bool OK_status;

    pItem = item(i, 1);
    auto profile_class = pItem->text().toInt(&OK_status);
    if (!OK_status)
    {
      std::cout << "Error in saving profile_class for Agent Profile: ["
                << profile_name << "]" << std::endl;
      return -1;
    }
    current_profile.profile_class = static_cast<size_t>(profile_class);

    pItem = item(i, 2);
    auto max_accel = pItem->text().toDouble(&OK_status);
    if (!OK_status)
    {
      std::cout << "Error in saving max_accel for Agent Profile: ["
                << profile_name << "]" << std::endl;
      return -1;
    }
    current_profile.max_accel = static_cast<double>(max_accel);

    pItem = item(i, 3);
    auto max_angle_vel = pItem->text().toDouble(&OK_status);
    if (!OK_status)
    {
      std::cout << "Error in saving max_angle_vel for Agent Profile: ["
                << profile_name << "]" << std::endl;
      return -1;
    }
    current_profile.max_angle_vel = static_cast<double>(max_angle_vel);

    pItem = item(i, 4);
    auto max_neighbors = pItem->text().toInt(&OK_status);
    if (!OK_status)
    {
      std::cout << "Error in saving max_neighbors for Agent Profile: ["
                << profile_name << "]" << std::endl;
      return -1;
    }
    current_profile.max_neighbors = static_cast<size_t>(max_neighbors);

    pItem = item(i, 5);
    auto max_speed = pItem->text().toDouble(&OK_status);
    if (!OK_status)
    {
      std::cout << "Error in saving max_speed for Agent Profile: ["
                << profile_name << "]" << std::endl;
      return -1;
    }
    current_profile.max_speed = static_cast<double>(max_speed);

    pItem = item(i, 6);
    auto neighbor_dist = pItem->text().toDouble(&OK_status);
    if (!OK_status)
    {
      std::cout << "Error in saving neighbor dist for Agent Profile: ["
                << profile_name << "]" << std::endl;
      return -1;
    }
    current_profile.neighbor_dist = static_cast<double>(neighbor_dist);

    pItem = item(i, 7);
    auto obstacle_set = pItem->text().toInt(&OK_status);
    if (!OK_status)
    {
      std::cout << "Error in saving obstacle_set for Agent Profile: ["
                << profile_name << "]" << std::endl;
      return -1;
    }
    current_profile.obstacle_set = static_cast<double>(obstacle_set);

    pItem = item(i, 8);
    auto pref_speed = pItem->text().toDouble(&OK_status);
    if (!OK_status)
    {
      std::cout << "Error in saving pref_speed for Agent Profile: ["
                << profile_name << "]" << std::endl;
      return -1;
    }
    current_profile.pref_speed = static_cast<double>(pref_speed);

    pItem = item(i, 9);
    auto r = pItem->text().toDouble(&OK_status);
    if (!OK_status)
    {
      std::cout << "Error in saving r for Agent Profile: ["
                << profile_name << "]" << std::endl;
      return -1;
    }
    current_profile.r = static_cast<double>(r);

    pItem = item(i, 10);
    auto ORCA_tau = pItem->text().toDouble(&OK_status);
    if (!OK_status)
    {
      std::cout << "Error in saving ORCA_tau for Agent Profile: ["
                << profile_name << "]" << std::endl;
      return -1;
    }
    current_profile.ORCA_tau = static_cast<double>(ORCA_tau);

    pItem = item(i, 11);
    auto ORCA_tauObst = pItem->text().toDouble(&OK_status);
    if (!OK_status)
    {
      std::cout << "Error in saving ORCA_tauObst for Agent Profile: ["
                << profile_name << "]" << std::endl;
      return -1;
    }
    current_profile.ORCA_tauObst = static_cast<double>(ORCA_tauObst);
  }
  return row_count-1;
}
