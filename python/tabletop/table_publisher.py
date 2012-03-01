"""
Module defining the Table Publisher
"""

from object_recognition_core.io.io_ros import Publisher_Marker, Publisher_MarkerArray
from object_recognition_core.io.sink import Sink
from tabletop_table import TableMsgAssembler
import ecto
import ecto_tabletop

MarkerPub = Publisher_Marker
MarkerArrayPub = Publisher_MarkerArray

########################################################################################################################

class TablePublisher(ecto.BlackBox):
    """
    Class publishing the different results of tabletop
    """
    _table_msg_assembler = TableMsgAssembler
    _marker_array_hull_ = MarkerArrayPub
    _marker_array_origin_ = MarkerArrayPub
    _marker_array_table_ = MarkerArrayPub
    _marker_array_delete = MarkerArrayPub
    _marker_array_clusters = MarkerArrayPub

    def declare_params(self, p):
        p.declare('marker_hull_topic', 'The ROS topic to use for the table message.', 'marker_hull')
        p.declare('marker_origin_topic', 'The ROS topic to use for the table message.', 'marker_origin')
        p.declare('marker_table_topic', 'The ROS topic to use for the table message.', 'marker_table')
        p.declare('marker_array_delete', 'The ROS topic to use for the markers to remove.', 'marker_array_delete')
        p.declare('marker_array_clusters', 'The ROS topic to use for the markers of the clusters.', 'marker_array_clusters')
        p.declare('latched', 'Determines if the topics will be latched.', True)

    def declare_io(self, _p, i, _o):
        i.forward_all('_table_msg_assembler')

    def configure(self, p, _i, _o):
        self._table_msg_assembler = TablePublisher._table_msg_assembler()
        self._marker_array_hull_ = TablePublisher._marker_array_hull_(topic_name=p.marker_hull_topic, latched=p.latched)
        self._marker_array_origin_ = TablePublisher._marker_array_origin_(topic_name=p.marker_origin_topic, latched=p.latched)
        self._marker_array_table_ = TablePublisher._marker_array_table_(topic_name=p.marker_table_topic, latched=p.latched)
        self._marker_array_delete = TablePublisher._marker_array_delete(topic_name=p.marker_array_delete)
        self._marker_array_clusters = TablePublisher._marker_array_clusters(topic_name=p.marker_array_clusters)

    def connections(self):
        return [self._table_msg_assembler['marker_array_hull'] >> self._marker_array_hull_[:],
                self._table_msg_assembler['marker_array_origin'] >> self._marker_array_origin_[:],
                self._table_msg_assembler['marker_array_table'] >> self._marker_array_table_[:],
                self._table_msg_assembler['marker_array_delete'] >> self._marker_array_delete[:],
                self._table_msg_assembler['marker_array_clusters'] >> self._marker_array_clusters[:] ]

########################################################################################################################

class TablePublisherSink(Sink):

    @classmethod
    def type_name(cls):
        return 'table_publisher'

    @classmethod
    def sink(self, *args, **kwargs):
        return TablePublisher()
