import requests
import json
from collections import OrderedDict
from decimal import Decimal
from io import BytesIO
from PIL import Image
import math
import elevation as dem
import os

def is_valid_type(element, cls):
    """
    Test if an element is of a given type.

    :param Element() element: The element instance to test
    :param Element cls: The element class to test
    :return: False or True
    :rtype: Boolean
    """
    return isinstance(element, cls) and element.id is not None

class Overpass(object):
    
    """
    Class to access the Overpass API
    """

    def __init__(self, overpass_server="http://overpass-api.de/api/interpreter"):
        self.overpass_server = overpass_server
        self.headers = {"User-Agent":"Mozilla/5.0 (Macintosh; Intel Mac OS X 10_15_4) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/83.0.4103.97 Safari/537.36"}

    def queryBuilder(bbox, tags=['building', 'highway'], types=['node', 'way', 'relation'], format='json'):
        '''
        QL template syntax :
        [out:json][bbox:ymin,xmin,ymax,xmax];(node[tag1];node[tag2];((way[tag1];way[tag2];);>;);relation;);out;
        '''

        # s,w,n,e <--> ymin,xmin,ymax,xmax
        bboxStr = ','.join(map(str, bbox))

        head = "[out:"+format+"][bbox:"+bboxStr+"];"

        union = '('
        # all tagged nodes
        if 'node' in types:
            if tags:
                union += ';'.join(['node['+tag+']' for tag in tags]) + ';'
            else:
                union += 'node;'

        # all tagged ways with all their nodes (recurse down)
        if 'way' in types:
            union += '(('
            if tags:
                union += ';'.join(['way['+tag+']' for tag in tags]) + ';);'
            else:
                union += 'way;);'
            union += '>;);'

        # all relations (no filter tag applied)
        if 'relation' in types or 'rel' in types:
            union += 'relation;'
        union += ')'

        output = ';out;'
        qry = head + union + output

        return qry


    def query(self, query):
        """
        Query the Overpass API

        :param String|Bytes query: The query string in Overpass QL
        :return: The parsed result
        :rtype: overpy.Result
        """
        if not isinstance(query, bytes):
            query = query.encode("utf-8")
        response = requests.get(self.overpass_server, params={'data': query}, headers=self.headers)
        return Result.from_json(response.json(), api=self)


    def deg2num(self, lat_deg, lon_deg, zoom):
        lat_rad = math.radians(lat_deg)
        n = 2.0 ** zoom
        xtile = int((lon_deg + 180.0) / 360.0 * n)
        ytile = int((1.0 - math.log(math.tan(lat_rad) +
                    (1 / math.cos(lat_rad))) / math.pi) / 2.0 * n)
        return (xtile, ytile)


    def num2deg(self, xtile, ytile, zoom):
        n = 2.0 ** zoom
        lon_deg = xtile / n * 360.0 - 180.0
        lat_rad = math.atan(math.sinh(math.pi * (1 - 2 * ytile / n)))
        lat_deg = math.degrees(lat_rad)
        return (lat_deg, lon_deg)


    def getImageTile(self, lat=51.88863727036334, lon=10.417070845502911, delta_lat=0.001,  delta_long=0.001, zoom=19):
        xmin, ymax = self.deg2num(lat, lon, zoom)
        xmax, ymin = self.deg2num(lat + delta_lat, lon + delta_long, zoom)

        Cluster = Image.new('RGB', ((xmax-xmin+1)*256-1, (ymax-ymin+1)*256-1))
        for xtile in range(xmin, xmax+1):
            for ytile in range(ymin,  ymax+1):
                try:
                    imgurl = f"http://tile.openstreetmap.org/{zoom}/{xtile}/{ytile}.png"
                    # print("Opening: " + imgurl)
                    imgstr = requests.get(imgurl, headers=self.headers)
                    tile = Image.open(BytesIO(imgstr.content))
                    Cluster.paste(tile, box=((xtile-xmin)*256,  (ytile-ymin)*255))
                except Exception as ex:
                    print(f"Couldn't download image: {ex}")
                    tile = None
                    
        # Cluster = getImageCluster(51.88863727036334, 10.417070845502911, 0.001,  0.001, 19)
        # fig = plt.figure(figsize=(16,16))
        # plt.imshow(np.asarray(Cluster))
        # plt.show()
        return Cluster


    def getBuildingsFootprints(self, roi=[49.7955670752582, 9.89987744122153, 49.802298332928636, 9.909039867216649], default_height:float=5.0):
		#bbx: [s,w,n,e] <--> [ymin,xmin,ymax,xmax]
        ql_query = f'[out:json][bbox: {roi[0]},{roi[1]},{roi[2]},{roi[3]}];(way["building"];relation["building"];);out body;>;out skel qt;'
        # print(ql_query)
        result = self.query(ql_query)
        buildings = []

        for way in result.ways:
            height = default_height  # Default height
            levels = 1  # Default building levels
            height_found = False
            building_levels_found = False
            for k, v in way.tags.items():
                k = k.lower()
                if 'height' in k:
                    height = float(v)
                    height_found = True
                elif 'building:level' in k:
                    levels = int(v)
                    building_levels_found = True

            if height_found:
                # If height tag is available give it a priority.
                height = 1 * height
                
            elif building_levels_found:
                # Otherwise, look for building:levels tag to calculate building height
                height = levels * height

            # extags = list(way.tags.keys()) + [k + '=' + v for k, v in way.tags.items()]
            pts = [(float(node.lon), float(node.lat), height) for node in way.nodes]
            buildings.append(pts)

        # Get elevation
        fname = f"{os.getcwd()}/ROI-DEM.tif" 
        dem.clip(bounds=(roi[0], roi[1], roi[2], roi[3]), output=fname, product='SRTM1');

        return buildings


class Result(object):
    
    """
    Class to handle the result.
    """

    def __init__(self, elements=None, api=None):
        """

        :param List elements:
        :param api:
        :type api: overpy.Overpass
        """
        if elements is None:
            elements = []
        self._nodes = OrderedDict((element.id, element) for element in elements if is_valid_type(element, Node))
        self._ways = OrderedDict((element.id, element) for element in elements if is_valid_type(element, Way))
        self._relations = OrderedDict((element.id, element) for element in elements if is_valid_type(element, Relation))
        self._class_collection_map = {Node: self._nodes, Way: self._ways, Relation: self._relations}
        self.api = api
        self._bounds = {}

    def expand(self, other):
        """
        Add all elements from an other result to the list of elements of this result object.

        It is used by the auto resolve feature.

        :param other: Expand the result with the elements from this result.
        :type other: overpy.Result
        :raises ValueError: If provided parameter is not instance of :class:`overpy.Result`
        """
        if not isinstance(other, Result):
            raise ValueError("Provided argument has to be instance of overpy:Result()")

        other_collection_map = {Node: other.nodes, Way: other.ways, Relation: other.relations}
        for element_type, own_collection in self._class_collection_map.items():
            for element in other_collection_map[element_type]:
                if is_valid_type(element, element_type) and element.id not in own_collection:
                    own_collection[element.id] = element

    def append(self, element):
        """
        Append a new element to the result.

        :param element: The element to append
        :type element: overpy.Element
        """
        if is_valid_type(element, Element):
            self._class_collection_map[element.__class__].setdefault(element.id, element)

    def get_elements(self, filter_cls, elem_id=None):
        """
        Get a list of elements from the result and filter the element type by a class.

        :param filter_cls:
        :param elem_id: ID of the object
        :type elem_id: Integer
        :return: List of available elements
        :rtype: List
        """
        result = []
        if elem_id is not None:
            try:
                result = [self._class_collection_map[filter_cls][elem_id]]
            except KeyError:
                result = []
        else:
            for e in self._class_collection_map[filter_cls].values():
                result.append(e)
        return result

    def get_ids(self, filter_cls):
        """

        :param filter_cls:
        :return:
        """
        return list(self._class_collection_map[filter_cls].keys())

    def get_node_ids(self):
        return self.get_ids(filter_cls=Node)

    def get_way_ids(self):
        return self.get_ids(filter_cls=Way)

    def get_relation_ids(self):
        return self.get_ids(filter_cls=Relation)

    @classmethod
    def from_json(cls, data, api=None):
        """
        Create a new instance and load data from json object.

        :param data: JSON data returned by the Overpass API
        :type data: Dict
        :param api:
        :type api: overpy.Overpass
        :return: New instance of Result object
        :rtype: overpy.Result
        """
        result = cls(api=api)
        for elem_cls in [Node, Way, Relation]:
            for element in data.get("elements", []):
                e_type = element.get("type")
                if hasattr(e_type, "lower") and e_type.lower() == elem_cls._type_value:
                    result.append(elem_cls.from_json(element, result=result))

        return result

    def get_node(self, node_id):
        """
        Get a node by its ID.

        :param node_id: The node ID
        :type node_id: Integer
        :return: The node
        :rtype: overpy.Node
        """
        nodes = self.get_nodes(node_id=node_id)
        return nodes[0]

    def get_nodes(self, node_id=None, **kwargs):
        """
        Alias for get_elements() but filter the result by Node()

        :param node_id: The Id of the node
        :type node_id: Integer
        :return: List of elements
        """
        return self.get_elements(Node, elem_id=node_id, **kwargs)

    def get_relation(self, rel_id):
        """
        Get a relation by its ID.

        :param rel_id: The relation ID
        :type rel_id: Integer
        :return: The relation
        """
        relations = self.get_relations(rel_id=rel_id)
        return relations[0]

    def get_relations(self, rel_id=None, **kwargs):
        """
        Alias for get_elements() but filter the result by Relation

        :param rel_id: Id of the relation
        :type rel_id: Integer
        :return: List of elements
        """
        return self.get_elements(Relation, elem_id=rel_id, **kwargs)

    def get_way(self, way_id):
        """
        Get a way by its ID.

        :param way_id: The way ID
        :type way_id: Integer
        :return: The way
        """
        ways = self.get_ways(way_id=way_id)
        return ways[0]

    def get_ways(self, way_id=None, **kwargs):
        """
        Alias for get_elements() but filter the result by Way

        :param way_id: The Id of the way
        :type way_id: Integer
        :return: List of elements
        """
        return self.get_elements(Way, elem_id=way_id, **kwargs)

    def get_bounds(self):
        if not self._bounds:
            lons, lats = zip(*[(e.lon, e.lat) for e in self._nodes.values()])
            self._bounds['minlon'] = float(min(lons))
            self._bounds['maxlon'] = float(max(lons))
            self._bounds['minlat'] = float(min(lats))
            self._bounds['maxlat'] = float(max(lats))
        return self._bounds

    node_ids = property(get_node_ids)
    nodes = property(get_nodes)
    relation_ids = property(get_relation_ids)
    relations = property(get_relations)
    way_ids = property(get_way_ids)
    ways = property(get_ways)
    bounds = property(get_bounds)


class Element(object):

    """
    Base element
    """

    def __init__(self, attributes=None, result=None, tags=None):
        """
        :param attributes: Additional attributes
        :type attributes: Dict
        :param result: The result object this element belongs to
        :param tags: List of tags
        :type tags: Dict
        """

        self._result = result
        self.attributes = attributes
        self.id = None
        self.tags = tags


class Node(Element):

    """
    Class to represent an element of type node
    """

    _type_value = "node"

    def __init__(self, node_id=None, lat=None, lon=None, **kwargs):
        """
        :param lat: Latitude
        :type lat: Decimal or Float
        :param lon: Longitude
        :type long: Decimal or Float
        :param node_id: Id of the node element
        :type node_id: Integer
        :param kwargs: Additional arguments are passed directly to the parent class
        """

        Element.__init__(self, **kwargs)
        self.id = node_id
        self.lat = lat
        self.lon = lon

    def __repr__(self):
        return f"<overpy.Node id={self.id} lat={self.lat} lon={self.lon}>"

    @classmethod
    def from_json(cls, data, result=None):
        """
        Create new Node element from JSON data

        :param data: Element data from JSON
        :type data: Dict
        :param result: The result this element belongs to
        :type result: overpy.Result
        :return: New instance of Node
        :rtype: overpy.Node
        :raises overpy.exception.ElementDataWrongType: If type value of the passed JSON data does not match.
        """
        if data.get("type") != cls._type_value:
            print(f"ElementDataWrongType: {cls._type_value}, ata.get('type')")

        tags = data.get("tags", {})

        node_id = data.get("id")
        lat = data.get("lat")
        lon = data.get("lon")

        attributes = {}
        ignore = ["type", "id", "lat", "lon", "tags"]
        for n, v in data.items():
            if n in ignore:
                continue
            attributes[n] = v

        return cls(node_id=node_id, lat=lat, lon=lon, tags=tags, attributes=attributes, result=result)


class Way(Element):

    """
    Class to represent an element of type way
    """

    _type_value = "way"

    def __init__(self, way_id=None, node_ids=None, **kwargs):
        """
        :param node_ids: List of node IDs
        :type node_ids: List or Tuple
        :param way_id: Id of the way element
        :type way_id: Integer
        :param kwargs: Additional arguments are passed directly to the parent class

        """

        Element.__init__(self, **kwargs)
        #: The id of the way
        self.id = way_id

        #: List of Ids of the associated nodes
        self._node_ids = node_ids

    def __repr__(self):
        return f"<overpy.Way id={self.id} nodes={self._node_ids}>"

    @property
    def nodes(self):
        """
        List of nodes associated with the way.
        """
        return self.get_nodes()

    def get_nodes(self):
        """
        Get the nodes defining the geometry of the way

        :param resolve_missing: Try to resolve missing nodes.
        :return: List of nodes
        :rtype: List of overpy.Node
        :raises overpy.exception.DataIncomplete: At least one referenced node is not available in the result cache.
        :raises overpy.exception.DataIncomplete: If resolve_missing is True and at least one node can't be resolved.
        """
        result = []

        for node_id in self._node_ids:
            try:
                node = self._result.get_node(node_id)
            except Exception as ex:
                node = None

            if node is not None:
                result.append(node)
                continue

            result.append(node)

        return result

    @classmethod
    def from_json(cls, data, result=None):
        """
        Create new Way element from JSON data

        :param data: Element data from JSON
        :type data: Dict
        :param result: The result this element belongs to
        :type result: overpy.Result
        :return: New instance of Way
        :rtype: overpy.Way
        :raises overpy.exception.ElementDataWrongType: If type value of the passed JSON data does not match.
        """
        if data.get("type") != cls._type_value:
            print(f"ElementDataWrongType: {cls._type_value}, ata.get('type')")
            
        tags = data.get("tags", {})

        way_id = data.get("id")
        node_ids = data.get("nodes")

        attributes = {}
        ignore = ["id", "nodes", "tags", "type"]
        for n, v in data.items():
            if n in ignore:
                continue
            attributes[n] = v

        return cls(way_id=way_id, attributes=attributes, node_ids=node_ids, tags=tags, result=result)

    
class Relation(Element):

    """
    Class to represent an element of type relation
    """

    _type_value = "relation"

    def __init__(self, rel_id=None, members=None, **kwargs):
        """
        :param members:
        :param rel_id: Id of the relation element
        :type rel_id: Integer
        :param kwargs:
        :return:
        """

        Element.__init__(self, **kwargs)
        self.id = rel_id
        self.members = members

    def __repr__(self):
        return f"<overpy.Relation id={self.id}>"

    @classmethod
    def from_json(cls, data, result=None):
        """
        Create new Relation element from JSON data

        :param data: Element data from JSON
        :type data: Dict
        :param result: The result this element belongs to
        :type result: overpy.Result
        :return: New instance of Relation
        :rtype: overpy.Relation
        :raises overpy.exception.ElementDataWrongType: If type value of the passed JSON data does not match.
        """
        if data.get("type") != cls._type_value:
            print(f"ElementDataWrongType: {cls._type_value}, ata.get('type')")

        tags = data.get("tags", {})

        rel_id = data.get("id")

        members = []

        supported_members = [RelationNode, RelationWay, RelationRelation]
        for member in data.get("members", []):
            type_value = member.get("type")
            for member_cls in supported_members:
                if member_cls._type_value == type_value:
                    members.append(
                        member_cls.from_json(
                            member,
                            result=result
                        )
                    )

        attributes = {}
        ignore = ["id", "members", "tags", "type"]
        for n, v in data.items():
            if n in ignore:
                continue
            attributes[n] = v

        return cls(rel_id=rel_id, attributes=attributes, members=members, tags=tags, result=result)

   
class RelationMember(object):

    """
    Base class to represent a member of a relation.
    """

    def __init__(self, ref=None, role=None, result=None):
        """
        :param ref: Reference Id
        :type ref: Integer
        :param role: The role of the relation member
        :type role: String
        :param result:
        """
        self.ref = ref
        self._result = result
        self.role = role

    @classmethod
    def from_json(cls, data, result=None):
        """
        Create new RelationMember element from JSON data

        :param child: Element data from JSON
        :type child: Dict
        :param result: The result this element belongs to
        :type result: overpy.Result
        :return: New instance of RelationMember
        :rtype: overpy.RelationMember
        :raises overpy.exception.ElementDataWrongType: If type value of the passed JSON data does not match.
        """
        if data.get("type") != cls._type_value:
            print(f"ElementDataWrongType: {cls._type_value}, ata.get('type')")

        ref = data.get("ref")
        role = data.get("role")
        return cls(ref=ref, role=role, result=result)

  
class RelationNode(RelationMember):
    _type_value = "node"

    def resolve(self):
        return self._result.get_node(self.ref)

    def __repr__(self):
        return f"<overpy.RelationNode ref={self.ref} role={self.role}>"


class RelationWay(RelationMember):
    _type_value = "way"

    def resolve(self):
        return self._result.get_way(self.ref)

    def __repr__(self):
        return f"<overpy.RelationWay ref={self.ref} role={self.role}>"


class RelationRelation(RelationMember):
    _type_value = "relation"

    def resolve(self):
        return self._result.get_relation(self.ref)

    def __repr__(self):
        return f"<overpy.RelationRelation ref={self.ref} role={self.role}>"
