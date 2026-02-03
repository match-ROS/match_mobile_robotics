import sys
import rospy
import rosgraph
from PyQt5 import QtWidgets, QtCore

class RosNodeMonitor(QtWidgets.QWidget):
    REFRESH_MS = 2000  # how often to poll [ms]

    def __init__(self, watch_nodes):
        """
        :param watch_nodes: list of node names to monitor, e.g. ['/twist_combiner_world', ...]
        """
        super().__init__()
        rospy.init_node('ros_node_monitor_gui', anonymous=True)
        self.master = rosgraph.masterapi.Master('/ros_node_monitor_gui')
        self.watch_nodes = watch_nodes

        # Set up table: columns: Node, Running?, Publishers, Subscribers
        self.table = QtWidgets.QTableWidget(len(watch_nodes), 4)
        self.table.setHorizontalHeaderLabels(['Node', 'Up?', 'Publishes', 'Subscribes'])
        for row, nm in enumerate(watch_nodes):
            self.table.setItem(row, 0, QtWidgets.QTableWidgetItem(nm))
        self.table.verticalHeader().setVisible(False)
        self.table.horizontalHeader().setStretchLastSection(True)

        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self.table)
        self.setLayout(layout)

        # Poll timer
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_status)
        self.timer.start(self.REFRESH_MS)
        self.update_status()

    def update_status(self):
        # getSystemState() → (publishers, subscribers, services)
        try:
            pubs, subs, services = self.master.getSystemState()
        except Exception as e:
            rospy.logwarn(f"Could not contact master: {e}")
            return

        # build topic→nodes mapping
        topic_pubs   = { t: nodes for t, nodes in pubs    }
        topic_subs   = { t: nodes for t, nodes in subs    }

        # build node→[topics] maps
        node_to_pubs = { n: [] for n in self.watch_nodes }
        node_to_subs = { n: [] for n in self.watch_nodes }

        # For each topic, for each node in publisher list, record it
        for topic, nodes in topic_pubs.items():
            for n in nodes:
                if n in node_to_pubs:
                    node_to_pubs[n].append(topic)
        for topic, nodes in topic_subs.items():
            for n in nodes:
                if n in node_to_subs:
                    node_to_subs[n].append(topic)

        # also pull list of all active nodes
        try:
            # active = set(self.master.getSystemState()[2])  # outdated; better: use getPid or rosnode API
            # # Actually, simple alternative: rosnode list
            # code2, msg2, node_list = self.master.getSystemState()
            # But getSystemState only lists topics; instead:
            node_list = set()
            for t,n_list in pubs + subs:
                node_list.update(n_list)
        except:
            node_list = set()

        # Update table rows
        for row, nm in enumerate(self.watch_nodes):
            is_up = nm in node_list
            self.table.setItem(row, 1, QtWidgets.QTableWidgetItem("✔" if is_up else "✘"))
            self.table.setItem(row, 2, QtWidgets.QTableWidgetItem("\n".join(node_to_pubs.get(nm, []))))
            self.table.setItem(row, 3, QtWidgets.QTableWidgetItem("\n".join(node_to_subs.get(nm, []))))

            # color the “Up?” cell
            item = self.table.item(row, 1)
            if is_up:
                item.setBackground(QtCore.Qt.green)
            else:
                item.setBackground(QtCore.Qt.red)

if __name__ == '__main__':
    # list the exact names from your launch file:
    watch = [
        '/path_index_advancer',
        '/twist_combiner_world',
        '/twist_combiner_profile_offset',
        '/tcp_in_base_ideal_publisher',
        '/twist_combiner',
        '/ur_vel_induced_by_mir',
        '/world_twist_in_mir',
        '/ur_direction_controller',
        '/pid_twist_controller_direction/pid_twist_controller_direction',
        '/pid_twist_controller_orthogonal/pid_twist_controller_orthogonal',

    ]
    app = QtWidgets.QApplication(sys.argv)
    w = RosNodeMonitor(watch)
    w.resize(800, 300)
    w.setWindowTitle("ROS Node Monitor")
    w.show()
    sys.exit(app.exec_())
