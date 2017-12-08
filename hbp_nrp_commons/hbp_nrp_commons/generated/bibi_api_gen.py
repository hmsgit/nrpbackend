# ---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
# This file is part of the Neurorobotics Platform software
# Copyright (C) 2014,2015,2016,2017 Human Brain Project
# https://www.humanbrainproject.eu
#
# The Human Brain Project is a European Commission funded project
# in the frame of the Horizon2020 FET Flagship plan.
# http://ec.europa.eu/programmes/horizon2020/en/h2020-section/fet-flagships
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
# ---LICENSE-END

# ./bibi_api_gen.py
# -*- coding: utf-8 -*-
# PyXB bindings for NM:d8bc1000793d9bf647fd97a1512e459e4ce45e64
# Generated 2017-12-18 10:35:48.062113 by PyXB version 1.2.4 using Python 2.7.12.final.0
# Namespace http://schemas.humanbrainproject.eu/SP10/2014/BIBI

from __future__ import unicode_literals
import pyxb
import pyxb.binding
import pyxb.binding.saxer
import io
import pyxb.utils.utility
import pyxb.utils.domutils
import sys
import pyxb.utils.six as _six

# Unique identifier for bindings created at the same time
_GenerationUID = pyxb.utils.utility.UniqueIdentifier('urn:uuid:d423998e-e3d6-11e7-a7c3-847beb4693fd')

# Version of PyXB used to generate the bindings
_PyXBVersion = '1.2.4'
# Generated bindings are not compatible across PyXB versions
if pyxb.__version__ != _PyXBVersion:
    raise pyxb.PyXBVersionError(_PyXBVersion)

# Import bindings for namespaces imported into schema
import pyxb.binding.datatypes

# NOTE: All namespace declarations are reserved within the binding
Namespace = pyxb.namespace.NamespaceForURI('http://schemas.humanbrainproject.eu/SP10/2014/BIBI', create_if_missing=True)
Namespace.configureCategories(['typeBinding', 'elementBinding'])

def CreateFromDocument (xml_text, default_namespace=None, location_base=None):
    """Parse the given XML and use the document element to create a
    Python instance.

    @param xml_text An XML document.  This should be data (Python 2
    str or Python 3 bytes), or a text (Python 2 unicode or Python 3
    str) in the L{pyxb._InputEncoding} encoding.

    @keyword default_namespace The L{pyxb.Namespace} instance to use as the
    default namespace where there is no default namespace in scope.
    If unspecified or C{None}, the namespace of the module containing
    this function will be used.

    @keyword location_base: An object to be recorded as the base of all
    L{pyxb.utils.utility.Location} instances associated with events and
    objects handled by the parser.  You might pass the URI from which
    the document was obtained.
    """

    if pyxb.XMLStyle_saxer != pyxb._XMLStyle:
        dom = pyxb.utils.domutils.StringToDOM(xml_text)
        return CreateFromDOM(dom.documentElement, default_namespace=default_namespace)
    if default_namespace is None:
        default_namespace = Namespace.fallbackNamespace()
    saxer = pyxb.binding.saxer.make_parser(fallback_namespace=default_namespace, location_base=location_base)
    handler = saxer.getContentHandler()
    xmld = xml_text
    if isinstance(xmld, _six.text_type):
        xmld = xmld.encode(pyxb._InputEncoding)
    saxer.parse(io.BytesIO(xmld))
    instance = handler.rootObject()
    return instance

def CreateFromDOM (node, default_namespace=None):
    """Create a Python instance from the given DOM node.
    The node tag must correspond to an element declaration in this module.

    @deprecated: Forcing use of DOM interface is unnecessary; use L{CreateFromDocument}."""
    if default_namespace is None:
        default_namespace = Namespace.fallbackNamespace()
    return pyxb.binding.basis.element.AnyCreateFromDOM(node, default_namespace)


# Atomic simple type: {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}SimulationMode
class SimulationMode (pyxb.binding.datatypes.string, pyxb.binding.basis.enumeration_mixin):

    """The supported simulation modes of the NRP"""

    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'SimulationMode')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 62, 2)
    _Documentation = 'The supported simulation modes of the NRP'
SimulationMode._CF_enumeration = pyxb.binding.facets.CF_enumeration(value_datatype=SimulationMode, enum_prefix=None)
SimulationMode.SynchronousNestSimulation = SimulationMode._CF_enumeration.addEnumeration(unicode_value='SynchronousNestSimulation', tag='SynchronousNestSimulation')
SimulationMode.SynchronousSpinnakerSimulation = SimulationMode._CF_enumeration.addEnumeration(unicode_value='SynchronousSpinnakerSimulation', tag='SynchronousSpinnakerSimulation')
SimulationMode.SynchronousMUSICNestSimulation = SimulationMode._CF_enumeration.addEnumeration(unicode_value='SynchronousMUSICNestSimulation', tag='SynchronousMUSICNestSimulation')
SimulationMode._InitializeFacetMap(SimulationMode._CF_enumeration)
Namespace.addCategoryObject('typeBinding', 'SimulationMode', SimulationMode)

# Atomic simple type: {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}TimeStep
class TimeStep (pyxb.binding.datatypes.positiveInteger):

    """The timestep type of the CLE. This is a positive number in milliseconds. The maximum allowed value is 1000."""

    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'TimeStep')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 85, 2)
    _Documentation = 'The timestep type of the CLE. This is a positive number in milliseconds. The maximum allowed value is 1000.'
TimeStep._CF_maxInclusive = pyxb.binding.facets.CF_maxInclusive(value_datatype=TimeStep, value=pyxb.binding.datatypes.positiveInteger(1000))
TimeStep._InitializeFacetMap(TimeStep._CF_maxInclusive)
Namespace.addCategoryObject('typeBinding', 'TimeStep', TimeStep)

# Atomic simple type: {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}SDFFilename
class SDFFilename (pyxb.binding.datatypes.string):

    """This type denotes a path to an SDF (or Zip) file"""

    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'SDFFilename')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 120, 2)
    _Documentation = 'This type denotes a path to an SDF (or Zip) file'
SDFFilename._CF_pattern = pyxb.binding.facets.CF_pattern()
SDFFilename._CF_pattern.addPattern(pattern='(storage:)?[a-zA-Z0-9\\._/]*\\.(sdf|zip)')
SDFFilename._InitializeFacetMap(SDFFilename._CF_pattern)
Namespace.addCategoryObject('typeBinding', 'SDFFilename', SDFFilename)

# Atomic simple type: {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}H5Filename
class H5Filename (pyxb.binding.datatypes.string):

    """This type denotes a path to an H5 file."""

    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'H5Filename')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 129, 2)
    _Documentation = 'This type denotes a path to an H5 file.'
H5Filename._CF_pattern = pyxb.binding.facets.CF_pattern()
H5Filename._CF_pattern.addPattern(pattern='[a-zA-Z0-9\\._/]*\\.h5')
H5Filename._InitializeFacetMap(H5Filename._CF_pattern)
Namespace.addCategoryObject('typeBinding', 'H5Filename', H5Filename)

# Atomic simple type: {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}PythonFilename
class PythonFilename (pyxb.binding.datatypes.string):

    """This type denotes a path to a Python file."""

    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'PythonFilename')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 138, 2)
    _Documentation = 'This type denotes a path to a Python file.'
PythonFilename._CF_pattern = pyxb.binding.facets.CF_pattern()
PythonFilename._CF_pattern.addPattern(pattern='(storage:)?[a-zA-Z0-9\\._/]*\\.py')
PythonFilename._InitializeFacetMap(PythonFilename._CF_pattern)
Namespace.addCategoryObject('typeBinding', 'PythonFilename', PythonFilename)

# Atomic simple type: {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}ScriptFilename
class ScriptFilename (pyxb.binding.datatypes.string):

    """This type denotes a path to a script file."""

    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'ScriptFilename')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 147, 2)
    _Documentation = 'This type denotes a path to a script file.'
ScriptFilename._CF_pattern = pyxb.binding.facets.CF_pattern()
ScriptFilename._CF_pattern.addPattern(pattern='[a-zA-Z0-9\\._/]*\\.sh')
ScriptFilename._InitializeFacetMap(ScriptFilename._CF_pattern)
Namespace.addCategoryObject('typeBinding', 'ScriptFilename', ScriptFilename)

# Atomic simple type: {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}ConfTypeEnumeration
class ConfTypeEnumeration (pyxb.binding.datatypes.string, pyxb.binding.basis.enumeration_mixin):

    """This enumeration lists the standard configuration types used in the NRP."""

    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'ConfTypeEnumeration')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 179, 2)
    _Documentation = 'This enumeration lists the standard configuration types used in the NRP.'
ConfTypeEnumeration._CF_enumeration = pyxb.binding.facets.CF_enumeration(value_datatype=ConfTypeEnumeration, enum_prefix=None)
ConfTypeEnumeration.retina = ConfTypeEnumeration._CF_enumeration.addEnumeration(unicode_value='retina', tag='retina')
ConfTypeEnumeration.brainvisualizer = ConfTypeEnumeration._CF_enumeration.addEnumeration(unicode_value='brainvisualizer', tag='brainvisualizer')
ConfTypeEnumeration._InitializeFacetMap(ConfTypeEnumeration._CF_enumeration)
Namespace.addCategoryObject('typeBinding', 'ConfTypeEnumeration', ConfTypeEnumeration)

# Atomic simple type: {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}NeuronTarget
class NeuronTarget (pyxb.binding.datatypes.string, pyxb.binding.basis.enumeration_mixin):

    """The target of a neural connection"""

    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'NeuronTarget')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 301, 2)
    _Documentation = 'The target of a neural connection'
NeuronTarget._CF_enumeration = pyxb.binding.facets.CF_enumeration(value_datatype=NeuronTarget, enum_prefix=None)
NeuronTarget.Inhibitory = NeuronTarget._CF_enumeration.addEnumeration(unicode_value='Inhibitory', tag='Inhibitory')
NeuronTarget.Excitatory = NeuronTarget._CF_enumeration.addEnumeration(unicode_value='Excitatory', tag='Excitatory')
NeuronTarget._InitializeFacetMap(NeuronTarget._CF_enumeration)
Namespace.addCategoryObject('typeBinding', 'NeuronTarget', NeuronTarget)

# Atomic simple type: {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}TemplatePattern
class TemplatePattern (pyxb.binding.datatypes.string):

    """A regular expression denoting simple arithmetic index computations based on an index called i"""

    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'TemplatePattern')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 690, 2)
    _Documentation = 'A regular expression denoting simple arithmetic index computations based on an index called i'
TemplatePattern._CF_pattern = pyxb.binding.facets.CF_pattern()
TemplatePattern._CF_pattern.addPattern(pattern='(\\(\\s*)*(i|\\d+)(\\s*(\\+|\\*)\\s*(\\(\\s*)*(i|\\d+)\\s*|\\))*')
TemplatePattern._InitializeFacetMap(TemplatePattern._CF_pattern)
Namespace.addCategoryObject('typeBinding', 'TemplatePattern', TemplatePattern)

# Atomic simple type: {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}DeviceType
class DeviceType (pyxb.binding.datatypes.string, pyxb.binding.basis.enumeration_mixin):

    """The device types supported by the CLE"""

    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'DeviceType')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 762, 2)
    _Documentation = 'The device types supported by the CLE'
DeviceType._CF_enumeration = pyxb.binding.facets.CF_enumeration(value_datatype=DeviceType, enum_prefix=None)
DeviceType.ACSource = DeviceType._CF_enumeration.addEnumeration(unicode_value='ACSource', tag='ACSource')
DeviceType.DCSource = DeviceType._CF_enumeration.addEnumeration(unicode_value='DCSource', tag='DCSource')
DeviceType.FixedFrequency = DeviceType._CF_enumeration.addEnumeration(unicode_value='FixedFrequency', tag='FixedFrequency')
DeviceType.LeakyIntegratorAlpha = DeviceType._CF_enumeration.addEnumeration(unicode_value='LeakyIntegratorAlpha', tag='LeakyIntegratorAlpha')
DeviceType.LeakyIntegratorExp = DeviceType._CF_enumeration.addEnumeration(unicode_value='LeakyIntegratorExp', tag='LeakyIntegratorExp')
DeviceType.NCSource = DeviceType._CF_enumeration.addEnumeration(unicode_value='NCSource', tag='NCSource')
DeviceType.Poisson = DeviceType._CF_enumeration.addEnumeration(unicode_value='Poisson', tag='Poisson')
DeviceType.SpikeRecorder = DeviceType._CF_enumeration.addEnumeration(unicode_value='SpikeRecorder', tag='SpikeRecorder')
DeviceType.PopulationRate = DeviceType._CF_enumeration.addEnumeration(unicode_value='PopulationRate', tag='PopulationRate')
DeviceType._InitializeFacetMap(DeviceType._CF_enumeration)
Namespace.addCategoryObject('typeBinding', 'DeviceType', DeviceType)

# Atomic simple type: {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}RobotTopicAddress
class RobotTopicAddress (pyxb.binding.datatypes.string):

    """This type denotes a valid address of a robot control topic"""

    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'RobotTopicAddress')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 843, 2)
    _Documentation = 'This type denotes a valid address of a robot control topic'
RobotTopicAddress._CF_pattern = pyxb.binding.facets.CF_pattern()
RobotTopicAddress._CF_pattern.addPattern(pattern='(/[a-zA-Z0-9_-]+)+(/)?')
RobotTopicAddress._InitializeFacetMap(RobotTopicAddress._CF_pattern)
Namespace.addCategoryObject('typeBinding', 'RobotTopicAddress', RobotTopicAddress)

# Union simple type: {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}BrainFilename
# superclasses pyxb.binding.datatypes.anySimpleType
class BrainFilename (pyxb.binding.basis.STD_union):

    """This denotes the supported file types for neural network models. The current version only supports Python or H5 files for neural networks."""

    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'BrainFilename')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 113, 2)
    _Documentation = 'This denotes the supported file types for neural network models. The current version only supports Python or H5 files for neural networks.'

    _MemberTypes = ( H5Filename, PythonFilename, )
BrainFilename._CF_pattern = pyxb.binding.facets.CF_pattern()
BrainFilename._CF_enumeration = pyxb.binding.facets.CF_enumeration(value_datatype=BrainFilename)
BrainFilename._InitializeFacetMap(BrainFilename._CF_pattern,
   BrainFilename._CF_enumeration)
Namespace.addCategoryObject('typeBinding', 'BrainFilename', BrainFilename)

# Union simple type: {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}ConfType
# superclasses pyxb.binding.datatypes.anySimpleType
class ConfType (pyxb.binding.basis.STD_union):

    """This type denotes a configuration type which can be a standard configuration type or a custom type. The latter is just any string."""

    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'ConfType')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 172, 2)
    _Documentation = 'This type denotes a configuration type which can be a standard configuration type or a custom type. The latter is just any string.'

    _MemberTypes = ( ConfTypeEnumeration, pyxb.binding.datatypes.string, )
ConfType._CF_pattern = pyxb.binding.facets.CF_pattern()
ConfType._CF_enumeration = pyxb.binding.facets.CF_enumeration(value_datatype=ConfType)
ConfType.retina = 'retina'                        # originally ConfTypeEnumeration.retina
ConfType.brainvisualizer = 'brainvisualizer'      # originally ConfTypeEnumeration.brainvisualizer
ConfType._InitializeFacetMap(ConfType._CF_pattern,
   ConfType._CF_enumeration)
Namespace.addCategoryObject('typeBinding', 'ConfType', ConfType)

# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}BIBIConfiguration with content type ELEMENT_ONLY
class BIBIConfiguration (pyxb.binding.basis.complexTypeDefinition):
    """This class represents the root of the BIBI configuration."""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'BIBIConfiguration')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 9, 2)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}timestep uses Python identifier timestep
    __timestep = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'timestep'), 'timestep', '__httpschemas_humanbrainproject_euSP102014BIBI_BIBIConfiguration_httpschemas_humanbrainproject_euSP102014BIBItimestep', False, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 14, 6), )

    
    timestep = property(__timestep.value, __timestep.set, None, 'If specified, the CLE uses a different timestep than the default timestep of 20ms. The timestep is specified in milliseconds and depicts the time between two successive loops of the CLE in simulation time.')

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}brainModel uses Python identifier brainModel
    __brainModel = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'brainModel'), 'brainModel', '__httpschemas_humanbrainproject_euSP102014BIBI_BIBIConfiguration_httpschemas_humanbrainproject_euSP102014BIBIbrainModel', False, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 19, 6), )

    
    brainModel = property(__brainModel.value, __brainModel.set, None, 'The brain model depicts a path to the neural network model.')

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}bodyModel uses Python identifier bodyModel
    __bodyModel = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'bodyModel'), 'bodyModel', '__httpschemas_humanbrainproject_euSP102014BIBI_BIBIConfiguration_httpschemas_humanbrainproject_euSP102014BIBIbodyModel', False, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 24, 6), )

    
    bodyModel = property(__bodyModel.value, __bodyModel.set, None, 'The path to the robot model that should be used. This can either be a path to an SDF model or a path to a zip file containing all required assets for a robot. This zip file must have a file model.sdf at the root of the archive.')

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}mode uses Python identifier mode
    __mode = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'mode'), 'mode', '__httpschemas_humanbrainproject_euSP102014BIBI_BIBIConfiguration_httpschemas_humanbrainproject_euSP102014BIBImode', False, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 29, 6), )

    
    mode = property(__mode.value, __mode.set, None, 'The simulation mode. This determines the choice of the neural network simulator.')

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}extRobotController uses Python identifier extRobotController
    __extRobotController = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'extRobotController'), 'extRobotController', '__httpschemas_humanbrainproject_euSP102014BIBI_BIBIConfiguration_httpschemas_humanbrainproject_euSP102014BIBIextRobotController', False, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 34, 6), )

    
    extRobotController = property(__extRobotController.value, __extRobotController.set, None, 'A path to an external robot controller. If specified, the robot controller is started when the simulation begins and stopped when the simulation is over. Therefore, the path must be a path to a shell script that offers a function start and a function stop.')

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}configuration uses Python identifier configuration
    __configuration = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'configuration'), 'configuration', '__httpschemas_humanbrainproject_euSP102014BIBI_BIBIConfiguration_httpschemas_humanbrainproject_euSP102014BIBIconfiguration', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 39, 6), )

    
    configuration = property(__configuration.value, __configuration.set, None, 'The configuration entries of an experiment depict additional files required for the simulation of experiments using this BIBI configuration.')

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}connectors uses Python identifier connectors
    __connectors = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'connectors'), 'connectors', '__httpschemas_humanbrainproject_euSP102014BIBI_BIBIConfiguration_httpschemas_humanbrainproject_euSP102014BIBIconnectors', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 44, 6), )

    
    connectors = property(__connectors.value, __connectors.set, None, 'A list of connectors. This can be useful when specifying transfer functions ')

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}synapseDynamics uses Python identifier synapseDynamics
    __synapseDynamics = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'synapseDynamics'), 'synapseDynamics', '__httpschemas_humanbrainproject_euSP102014BIBI_BIBIConfiguration_httpschemas_humanbrainproject_euSP102014BIBIsynapseDynamics', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 49, 6), )

    
    synapseDynamics = property(__synapseDynamics.value, __synapseDynamics.set, None, 'A list of synapse dynamics. Such a synapse dynamic can be referenced later on in neural network devices.')

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}transferFunction uses Python identifier transferFunction
    __transferFunction = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'transferFunction'), 'transferFunction', '__httpschemas_humanbrainproject_euSP102014BIBI_BIBIConfiguration_httpschemas_humanbrainproject_euSP102014BIBItransferFunction', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 54, 6), )

    
    transferFunction = property(__transferFunction.value, __transferFunction.set, None, 'The transfer functions that are used to couple a neural network to robot')

    _ElementMap.update({
        __timestep.name() : __timestep,
        __brainModel.name() : __brainModel,
        __bodyModel.name() : __bodyModel,
        __mode.name() : __mode,
        __extRobotController.name() : __extRobotController,
        __configuration.name() : __configuration,
        __connectors.name() : __connectors,
        __synapseDynamics.name() : __synapseDynamics,
        __transferFunction.name() : __transferFunction
    })
    _AttributeMap.update({
        
    })
Namespace.addCategoryObject('typeBinding', 'BIBIConfiguration', BIBIConfiguration)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}BrainModel with content type ELEMENT_ONLY
class BrainModel (pyxb.binding.basis.complexTypeDefinition):
    """A neural network description as used in the CLE"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'BrainModel')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 94, 2)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}file uses Python identifier file
    __file = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'file'), 'file', '__httpschemas_humanbrainproject_euSP102014BIBI_BrainModel_httpschemas_humanbrainproject_euSP102014BIBIfile', False, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 99, 6), )

    
    file = property(__file.value, __file.set, None, 'A path to the neural network file.')

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}populations uses Python identifier populations
    __populations = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'populations'), 'populations', '__httpschemas_humanbrainproject_euSP102014BIBI_BrainModel_httpschemas_humanbrainproject_euSP102014BIBIpopulations', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 104, 6), )

    
    populations = property(__populations.value, __populations.set, None, "The populations in this field are the explicitly defined populations. Each of this population is defined as a view of an assumed 'circuit' population.")

    _ElementMap.update({
        __file.name() : __file,
        __populations.name() : __populations
    })
    _AttributeMap.update({
        
    })
Namespace.addCategoryObject('typeBinding', 'BrainModel', BrainModel)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}TransferFunction with content type EMPTY
class TransferFunction (pyxb.binding.basis.complexTypeDefinition):
    """This is the abstract type for a transfer function specification. A transfer function may be specified either in XML or in Python. These specification options are reflected in subclasses of the abstract transfer function type."""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
    _Abstract = True
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'TransferFunction')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 190, 2)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    _ElementMap.update({
        
    })
    _AttributeMap.update({
        
    })
Namespace.addCategoryObject('typeBinding', 'TransferFunction', TransferFunction)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}NeuronConnector with content type EMPTY
class NeuronConnector (pyxb.binding.basis.complexTypeDefinition):
    """This type denotes a connector to other populations"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
    _Abstract = True
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'NeuronConnector')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 319, 2)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Attribute weights uses Python identifier weights
    __weights = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'weights'), 'weights', '__httpschemas_humanbrainproject_euSP102014BIBI_NeuronConnector_weights', pyxb.binding.datatypes.double)
    __weights._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 323, 4)
    __weights._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 323, 4)
    
    weights = property(__weights.value, __weights.set, None, 'The weights of the connector denote the connections between the source neuron and the target neurons. If no weight is specified, the default weight of the neuron connection device is used.')

    
    # Attribute delays uses Python identifier delays
    __delays = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'delays'), 'delays', '__httpschemas_humanbrainproject_euSP102014BIBI_NeuronConnector_delays', pyxb.binding.datatypes.double)
    __delays._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 328, 4)
    __delays._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 328, 4)
    
    delays = property(__delays.value, __delays.set, None, 'The delays of the connector denote the delays of spike deliveries. If no delays are specified, the default delays of the neuron connection device is used.')

    
    # Attribute name uses Python identifier name
    __name = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'name'), 'name', '__httpschemas_humanbrainproject_euSP102014BIBI_NeuronConnector_name', pyxb.binding.datatypes.string, unicode_default='default')
    __name._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 333, 4)
    __name._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 333, 4)
    
    name = property(__name.value, __name.set, None, 'The name of the connector for later reference.')

    _ElementMap.update({
        
    })
    _AttributeMap.update({
        __weights.name() : __weights,
        __delays.name() : __delays,
        __name.name() : __name
    })
Namespace.addCategoryObject('typeBinding', 'NeuronConnector', NeuronConnector)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}NeuronConnectorRef with content type EMPTY
class NeuronConnectorRef (pyxb.binding.basis.complexTypeDefinition):
    """A reference to an elsewhere defined neural connector"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'NeuronConnectorRef')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 340, 2)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Attribute ref uses Python identifier ref
    __ref = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'ref'), 'ref', '__httpschemas_humanbrainproject_euSP102014BIBI_NeuronConnectorRef_ref', pyxb.binding.datatypes.string, required=True)
    __ref._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 346, 8)
    __ref._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 346, 8)
    
    ref = property(__ref.value, __ref.set, None, 'The name of the referenced connector')

    _ElementMap.update({
        
    })
    _AttributeMap.update({
        __ref.name() : __ref
    })
Namespace.addCategoryObject('typeBinding', 'NeuronConnectorRef', NeuronConnectorRef)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}SynapseDynamics with content type EMPTY
class SynapseDynamics (pyxb.binding.basis.complexTypeDefinition):
    """This type denotes a reusable synapse dynamics configuration"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
    _Abstract = True
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'SynapseDynamics')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 386, 2)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Attribute name uses Python identifier name
    __name = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'name'), 'name', '__httpschemas_humanbrainproject_euSP102014BIBI_SynapseDynamics_name', pyxb.binding.datatypes.string, unicode_default='default')
    __name._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 390, 4)
    __name._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 390, 4)
    
    name = property(__name.value, __name.set, None, 'The name of the synapse dynamics configuration')

    _ElementMap.update({
        
    })
    _AttributeMap.update({
        __name.name() : __name
    })
Namespace.addCategoryObject('typeBinding', 'SynapseDynamics', SynapseDynamics)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}SynapseDynamicsRef with content type EMPTY
class SynapseDynamicsRef (pyxb.binding.basis.complexTypeDefinition):
    """This type specifies a reference to a synapse dynamics configuration"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'SynapseDynamicsRef')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 397, 2)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Attribute ref uses Python identifier ref
    __ref = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'ref'), 'ref', '__httpschemas_humanbrainproject_euSP102014BIBI_SynapseDynamicsRef_ref', pyxb.binding.datatypes.string, required=True)
    __ref._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 403, 8)
    __ref._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 403, 8)
    
    ref = property(__ref.value, __ref.set, None, 'The name of the synapse dynamics configuration')

    _ElementMap.update({
        
    })
    _AttributeMap.update({
        __ref.name() : __ref
    })
Namespace.addCategoryObject('typeBinding', 'SynapseDynamicsRef', SynapseDynamicsRef)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}NeuronGroupSelector with content type EMPTY
class NeuronGroupSelector (pyxb.binding.basis.complexTypeDefinition):
    """This type denotes an abstract group of neurons"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
    _Abstract = True
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'NeuronGroupSelector')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 539, 2)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    _ElementMap.update({
        
    })
    _AttributeMap.update({
        
    })
Namespace.addCategoryObject('typeBinding', 'NeuronGroupSelector', NeuronGroupSelector)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}NeuronSelector with content type EMPTY
class NeuronSelector (pyxb.binding.basis.complexTypeDefinition):
    """The abstract base class of neuron selectors"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
    _Abstract = True
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'NeuronSelector')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 596, 2)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Attribute population uses Python identifier population
    __population = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'population'), 'population', '__httpschemas_humanbrainproject_euSP102014BIBI_NeuronSelector_population', pyxb.binding.datatypes.string, required=True)
    __population._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 600, 4)
    __population._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 600, 4)
    
    population = property(__population.value, __population.set, None, 'The population this neuron selector refers to')

    _ElementMap.update({
        
    })
    _AttributeMap.update({
        __population.name() : __population
    })
Namespace.addCategoryObject('typeBinding', 'NeuronSelector', NeuronSelector)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}NeuronSelectorTemplate with content type EMPTY
class NeuronSelectorTemplate (pyxb.binding.basis.complexTypeDefinition):
    """A template for neuron selectors"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
    _Abstract = True
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'NeuronSelectorTemplate')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 699, 2)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    _ElementMap.update({
        
    })
    _AttributeMap.update({
        
    })
Namespace.addCategoryObject('typeBinding', 'NeuronSelectorTemplate', NeuronSelectorTemplate)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}FlowExpression with content type EMPTY
class FlowExpression (pyxb.binding.basis.complexTypeDefinition):
    """The abstract base class for an information flow expression. In the scope of the Transfer functions, an information flow is an expression without any control flow."""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
    _Abstract = True
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'FlowExpression')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 852, 2)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    _ElementMap.update({
        
    })
    _AttributeMap.update({
        
    })
Namespace.addCategoryObject('typeBinding', 'FlowExpression', FlowExpression)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Argument with content type ELEMENT_ONLY
class Argument (pyxb.binding.basis.complexTypeDefinition):
    """A named argument"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'Argument')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 989, 2)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}value uses Python identifier value_
    __value = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'value'), 'value_', '__httpschemas_humanbrainproject_euSP102014BIBI_Argument_httpschemas_humanbrainproject_euSP102014BIBIvalue', False, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 994, 6), )

    
    value_ = property(__value.value, __value.set, None, 'The value passed for this argument')

    
    # Attribute name uses Python identifier name
    __name = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'name'), 'name', '__httpschemas_humanbrainproject_euSP102014BIBI_Argument_name', pyxb.binding.datatypes.string, required=True)
    __name._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 1000, 4)
    __name._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 1000, 4)
    
    name = property(__name.value, __name.set, None, 'The name of this argument')

    _ElementMap.update({
        __value.name() : __value
    })
    _AttributeMap.update({
        __name.name() : __name
    })
Namespace.addCategoryObject('typeBinding', 'Argument', Argument)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Local with content type ELEMENT_ONLY
class Local (pyxb.binding.basis.complexTypeDefinition):
    """A local variable"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'Local')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 1007, 2)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}body uses Python identifier body
    __body = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'body'), 'body', '__httpschemas_humanbrainproject_euSP102014BIBI_Local_httpschemas_humanbrainproject_euSP102014BIBIbody', False, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 1012, 6), )

    
    body = property(__body.value, __body.set, None, 'The initial value for this local variable')

    
    # Attribute name uses Python identifier name
    __name = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'name'), 'name', '__httpschemas_humanbrainproject_euSP102014BIBI_Local_name', pyxb.binding.datatypes.string, required=True)
    __name._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 1018, 4)
    __name._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 1018, 4)
    
    name = property(__name.value, __name.set, None, 'The name of the local variable')

    _ElementMap.update({
        __body.name() : __body
    })
    _AttributeMap.update({
        __name.name() : __name
    })
Namespace.addCategoryObject('typeBinding', 'Local', Local)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}PythonTransferFunction with content type MIXED
class PythonTransferFunction (TransferFunction):
    """This type denotes a transfer function entirely specified in the Python DSL PyTF."""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_MIXED
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'PythonTransferFunction')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 196, 2)
    _ElementMap = TransferFunction._ElementMap.copy()
    _AttributeMap = TransferFunction._AttributeMap.copy()
    # Base type is TransferFunction
    
    # Attribute src uses Python identifier src
    __src = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'src'), 'src', '__httpschemas_humanbrainproject_euSP102014BIBI_PythonTransferFunction_src', PythonFilename)
    __src._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 207, 8)
    __src._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 207, 8)
    
    src = property(__src.value, __src.set, None, "The 'src' attribute denotes the path of a python file that contains the entire transfer function. If this attribute is present, the actual contents of the transfer function element is ignored and only the contents of the specified Python file are taken into account.")

    _HasWildcardElement = True
    _ElementMap.update({
        
    })
    _AttributeMap.update({
        __src.name() : __src
    })
Namespace.addCategoryObject('typeBinding', 'PythonTransferFunction', PythonTransferFunction)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}BIBITransferFunction with content type ELEMENT_ONLY
class BIBITransferFunction (TransferFunction):
    """This type denotes the abstract base type of Transfer Functions specified entirely in the BIBI model, in XML"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = True
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'BIBITransferFunction')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 217, 2)
    _ElementMap = TransferFunction._ElementMap.copy()
    _AttributeMap = TransferFunction._AttributeMap.copy()
    # Base type is TransferFunction
    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}local uses Python identifier local
    __local = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'local'), 'local', '__httpschemas_humanbrainproject_euSP102014BIBI_BIBITransferFunction_httpschemas_humanbrainproject_euSP102014BIBIlocal', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 224, 10), )

    
    local = property(__local.value, __local.set, None, 'This denotes the local variables of this transfer function.')

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}device uses Python identifier device
    __device = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'device'), 'device', '__httpschemas_humanbrainproject_euSP102014BIBI_BIBITransferFunction_httpschemas_humanbrainproject_euSP102014BIBIdevice', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 230, 12), )

    
    device = property(__device.value, __device.set, None, 'This denotes device channels, connections of the transfer function to the neural network using exactly one device.')

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}deviceGroup uses Python identifier deviceGroup
    __deviceGroup = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'deviceGroup'), 'deviceGroup', '__httpschemas_humanbrainproject_euSP102014BIBI_BIBITransferFunction_httpschemas_humanbrainproject_euSP102014BIBIdeviceGroup', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 235, 12), )

    
    deviceGroup = property(__deviceGroup.value, __deviceGroup.set, None, 'This denotes the device group channels, connections of transfer functions to the neural network using a one-dimensional array of devices.')

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}topic uses Python identifier topic
    __topic = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'topic'), 'topic', '__httpschemas_humanbrainproject_euSP102014BIBI_BIBITransferFunction_httpschemas_humanbrainproject_euSP102014BIBItopic', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 241, 10), )

    
    topic = property(__topic.value, __topic.set, None, 'This denotes the connections of the transfer function to robot control channels.')

    
    # Attribute name uses Python identifier name
    __name = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'name'), 'name', '__httpschemas_humanbrainproject_euSP102014BIBI_BIBITransferFunction_name', pyxb.binding.datatypes.string, required=True)
    __name._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 247, 8)
    __name._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 247, 8)
    
    name = property(__name.value, __name.set, None, 'The name of the transfer function. This is used to identify the transfer function in order to update or delete it in a running simulation.')

    _ElementMap.update({
        __local.name() : __local,
        __device.name() : __device,
        __deviceGroup.name() : __deviceGroup,
        __topic.name() : __topic
    })
    _AttributeMap.update({
        __name.name() : __name
    })
Namespace.addCategoryObject('typeBinding', 'BIBITransferFunction', BIBITransferFunction)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}OneToOneConnector with content type EMPTY
class OneToOneConnector (NeuronConnector):
    """
        This connector type is obsolete.
      """
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'OneToOneConnector')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 355, 2)
    _ElementMap = NeuronConnector._ElementMap.copy()
    _AttributeMap = NeuronConnector._AttributeMap.copy()
    # Base type is NeuronConnector
    
    # Attribute weights inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}NeuronConnector
    
    # Attribute delays inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}NeuronConnector
    
    # Attribute name inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}NeuronConnector
    _ElementMap.update({
        
    })
    _AttributeMap.update({
        
    })
Namespace.addCategoryObject('typeBinding', 'OneToOneConnector', OneToOneConnector)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}AllToAllConnector with content type EMPTY
class AllToAllConnector (NeuronConnector):
    """This connector type is obsolete."""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'AllToAllConnector')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 366, 2)
    _ElementMap = NeuronConnector._ElementMap.copy()
    _AttributeMap = NeuronConnector._AttributeMap.copy()
    # Base type is NeuronConnector
    
    # Attribute weights inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}NeuronConnector
    
    # Attribute delays inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}NeuronConnector
    
    # Attribute name inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}NeuronConnector
    _ElementMap.update({
        
    })
    _AttributeMap.update({
        
    })
Namespace.addCategoryObject('typeBinding', 'AllToAllConnector', AllToAllConnector)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}FixedNumberPreConnector with content type EMPTY
class FixedNumberPreConnector (NeuronConnector):
    """This connector type is obsolete."""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'FixedNumberPreConnector')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 375, 2)
    _ElementMap = NeuronConnector._ElementMap.copy()
    _AttributeMap = NeuronConnector._AttributeMap.copy()
    # Base type is NeuronConnector
    
    # Attribute weights inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}NeuronConnector
    
    # Attribute delays inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}NeuronConnector
    
    # Attribute name inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}NeuronConnector
    
    # Attribute count uses Python identifier count
    __count = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'count'), 'count', '__httpschemas_humanbrainproject_euSP102014BIBI_FixedNumberPreConnector_count', pyxb.binding.datatypes.positiveInteger, required=True)
    __count._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 381, 8)
    __count._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 381, 8)
    
    count = property(__count.value, __count.set, None, None)

    _ElementMap.update({
        
    })
    _AttributeMap.update({
        __count.name() : __count
    })
Namespace.addCategoryObject('typeBinding', 'FixedNumberPreConnector', FixedNumberPreConnector)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}TsodyksMarkramMechanism with content type EMPTY
class TsodyksMarkramMechanism (SynapseDynamics):
    """A synapse dynamics implementation based on the Tsodyks-Markram mechanism"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'TsodyksMarkramMechanism')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 412, 2)
    _ElementMap = SynapseDynamics._ElementMap.copy()
    _AttributeMap = SynapseDynamics._AttributeMap.copy()
    # Base type is SynapseDynamics
    
    # Attribute name inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}SynapseDynamics
    
    # Attribute u uses Python identifier u
    __u = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'u'), 'u', '__httpschemas_humanbrainproject_euSP102014BIBI_TsodyksMarkramMechanism_u', pyxb.binding.datatypes.double, required=True)
    __u._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 418, 8)
    __u._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 418, 8)
    
    u = property(__u.value, __u.set, None, None)

    
    # Attribute tau_rec uses Python identifier tau_rec
    __tau_rec = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'tau_rec'), 'tau_rec', '__httpschemas_humanbrainproject_euSP102014BIBI_TsodyksMarkramMechanism_tau_rec', pyxb.binding.datatypes.double, required=True)
    __tau_rec._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 419, 8)
    __tau_rec._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 419, 8)
    
    tau_rec = property(__tau_rec.value, __tau_rec.set, None, None)

    
    # Attribute tau_facil uses Python identifier tau_facil
    __tau_facil = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'tau_facil'), 'tau_facil', '__httpschemas_humanbrainproject_euSP102014BIBI_TsodyksMarkramMechanism_tau_facil', pyxb.binding.datatypes.double, required=True)
    __tau_facil._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 420, 8)
    __tau_facil._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 420, 8)
    
    tau_facil = property(__tau_facil.value, __tau_facil.set, None, None)

    _ElementMap.update({
        
    })
    _AttributeMap.update({
        __u.name() : __u,
        __tau_rec.name() : __tau_rec,
        __tau_facil.name() : __tau_facil
    })
Namespace.addCategoryObject('typeBinding', 'TsodyksMarkramMechanism', TsodyksMarkramMechanism)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}DeviceChannel with content type ELEMENT_ONLY
class DeviceChannel (pyxb.binding.basis.complexTypeDefinition):
    """This type denotes a connection of a transfer function to a neural network"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'DeviceChannel')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 425, 2)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}neurons uses Python identifier neurons
    __neurons = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'neurons'), 'neurons', '__httpschemas_humanbrainproject_euSP102014BIBI_DeviceChannel_httpschemas_humanbrainproject_euSP102014BIBIneurons', False, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 430, 6), )

    
    neurons = property(__neurons.value, __neurons.set, None, 'This specifies the neurons that should be connected to this neural connector device')

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}connector uses Python identifier connector
    __connector = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'connector'), 'connector', '__httpschemas_humanbrainproject_euSP102014BIBI_DeviceChannel_httpschemas_humanbrainproject_euSP102014BIBIconnector', False, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 436, 8), )

    
    connector = property(__connector.value, __connector.set, None, 'Additional information on the connection to the neurons')

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}connectorRef uses Python identifier connectorRef
    __connectorRef = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'connectorRef'), 'connectorRef', '__httpschemas_humanbrainproject_euSP102014BIBI_DeviceChannel_httpschemas_humanbrainproject_euSP102014BIBIconnectorRef', False, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 441, 8), )

    
    connectorRef = property(__connectorRef.value, __connectorRef.set, None, 'A reference to a reusable connector')

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}synapseDynamics uses Python identifier synapseDynamics
    __synapseDynamics = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'synapseDynamics'), 'synapseDynamics', '__httpschemas_humanbrainproject_euSP102014BIBI_DeviceChannel_httpschemas_humanbrainproject_euSP102014BIBIsynapseDynamics', False, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 448, 8), )

    
    synapseDynamics = property(__synapseDynamics.value, __synapseDynamics.set, None, 'Additional information on the dynamics of the connection of this device to the neural network')

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}synapseDynamicsRef uses Python identifier synapseDynamicsRef
    __synapseDynamicsRef = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'synapseDynamicsRef'), 'synapseDynamicsRef', '__httpschemas_humanbrainproject_euSP102014BIBI_DeviceChannel_httpschemas_humanbrainproject_euSP102014BIBIsynapseDynamicsRef', False, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 453, 8), )

    
    synapseDynamicsRef = property(__synapseDynamicsRef.value, __synapseDynamicsRef.set, None, 'A reference to a reusable synapse dynamics')

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}target uses Python identifier target
    __target = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'target'), 'target', '__httpschemas_humanbrainproject_euSP102014BIBI_DeviceChannel_httpschemas_humanbrainproject_euSP102014BIBItarget', False, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 459, 6), )

    
    target = property(__target.value, __target.set, None, 'The target of this connection. This configuration is useful in particular for spike source devices such as Poisson generators. By default, these devices are excitatory but they can be configured to inhibit connected neurons.')

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}body uses Python identifier body
    __body = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'body'), 'body', '__httpschemas_humanbrainproject_euSP102014BIBI_DeviceChannel_httpschemas_humanbrainproject_euSP102014BIBIbody', False, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 464, 6), )

    
    body = property(__body.value, __body.set, None, 'This element is only meaningful for spike sources. It depicts the value to which the device should be configured.')

    
    # Attribute name uses Python identifier name
    __name = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'name'), 'name', '__httpschemas_humanbrainproject_euSP102014BIBI_DeviceChannel_name', pyxb.binding.datatypes.string, required=True)
    __name._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 470, 4)
    __name._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 470, 4)
    
    name = property(__name.value, __name.set, None, 'The name of this device channel')

    
    # Attribute type uses Python identifier type
    __type = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'type'), 'type', '__httpschemas_humanbrainproject_euSP102014BIBI_DeviceChannel_type', DeviceType, required=True)
    __type._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 475, 4)
    __type._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 475, 4)
    
    type = property(__type.value, __type.set, None, 'The type of the neural network connection specified with this device channel')

    _ElementMap.update({
        __neurons.name() : __neurons,
        __connector.name() : __connector,
        __connectorRef.name() : __connectorRef,
        __synapseDynamics.name() : __synapseDynamics,
        __synapseDynamicsRef.name() : __synapseDynamicsRef,
        __target.name() : __target,
        __body.name() : __body
    })
    _AttributeMap.update({
        __name.name() : __name,
        __type.name() : __type
    })
Namespace.addCategoryObject('typeBinding', 'DeviceChannel', DeviceChannel)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}DeviceGroupChannel with content type ELEMENT_ONLY
class DeviceGroupChannel (pyxb.binding.basis.complexTypeDefinition):
    """This type denotes a connection of a transfer function to a neural network using an array of devices"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'DeviceGroupChannel')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 482, 2)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}neurons uses Python identifier neurons
    __neurons = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'neurons'), 'neurons', '__httpschemas_humanbrainproject_euSP102014BIBI_DeviceGroupChannel_httpschemas_humanbrainproject_euSP102014BIBIneurons', False, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 487, 6), )

    
    neurons = property(__neurons.value, __neurons.set, None, 'This specifies the neurons that should be connected to this neural connector device')

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}connector uses Python identifier connector
    __connector = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'connector'), 'connector', '__httpschemas_humanbrainproject_euSP102014BIBI_DeviceGroupChannel_httpschemas_humanbrainproject_euSP102014BIBIconnector', False, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 493, 8), )

    
    connector = property(__connector.value, __connector.set, None, 'Additional information on the connection to the neurons')

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}connectorRef uses Python identifier connectorRef
    __connectorRef = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'connectorRef'), 'connectorRef', '__httpschemas_humanbrainproject_euSP102014BIBI_DeviceGroupChannel_httpschemas_humanbrainproject_euSP102014BIBIconnectorRef', False, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 498, 8), )

    
    connectorRef = property(__connectorRef.value, __connectorRef.set, None, 'A reference to a reusable connector')

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}synapseDynamics uses Python identifier synapseDynamics
    __synapseDynamics = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'synapseDynamics'), 'synapseDynamics', '__httpschemas_humanbrainproject_euSP102014BIBI_DeviceGroupChannel_httpschemas_humanbrainproject_euSP102014BIBIsynapseDynamics', False, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 505, 8), )

    
    synapseDynamics = property(__synapseDynamics.value, __synapseDynamics.set, None, 'Additional information on the dynamics of the connection of this device to the neural network')

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}synapseDynamicsRef uses Python identifier synapseDynamicsRef
    __synapseDynamicsRef = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'synapseDynamicsRef'), 'synapseDynamicsRef', '__httpschemas_humanbrainproject_euSP102014BIBI_DeviceGroupChannel_httpschemas_humanbrainproject_euSP102014BIBIsynapseDynamicsRef', False, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 510, 8), )

    
    synapseDynamicsRef = property(__synapseDynamicsRef.value, __synapseDynamicsRef.set, None, 'A reference to a reusable synapse dynamics')

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}target uses Python identifier target
    __target = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'target'), 'target', '__httpschemas_humanbrainproject_euSP102014BIBI_DeviceGroupChannel_httpschemas_humanbrainproject_euSP102014BIBItarget', False, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 516, 6), )

    
    target = property(__target.value, __target.set, None, 'The target of this connection. This configuration is useful in particular for spike source devices such as Poisson generators. By default, these devices are excitatory but they can be configured to inhibit connected neurons.')

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}body uses Python identifier body
    __body = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'body'), 'body', '__httpschemas_humanbrainproject_euSP102014BIBI_DeviceGroupChannel_httpschemas_humanbrainproject_euSP102014BIBIbody', False, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 521, 6), )

    
    body = property(__body.value, __body.set, None, 'This element is only meaningful for spike sources. It depicts the value to which the device should be configured.')

    
    # Attribute name uses Python identifier name
    __name = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'name'), 'name', '__httpschemas_humanbrainproject_euSP102014BIBI_DeviceGroupChannel_name', pyxb.binding.datatypes.string, required=True)
    __name._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 527, 4)
    __name._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 527, 4)
    
    name = property(__name.value, __name.set, None, 'The name of this device group channel')

    
    # Attribute type uses Python identifier type
    __type = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'type'), 'type', '__httpschemas_humanbrainproject_euSP102014BIBI_DeviceGroupChannel_type', DeviceType, required=True)
    __type._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 532, 4)
    __type._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 532, 4)
    
    type = property(__type.value, __type.set, None, 'The type of the neural network connection specified with this device group channel')

    _ElementMap.update({
        __neurons.name() : __neurons,
        __connector.name() : __connector,
        __connectorRef.name() : __connectorRef,
        __synapseDynamics.name() : __synapseDynamics,
        __synapseDynamicsRef.name() : __synapseDynamicsRef,
        __target.name() : __target,
        __body.name() : __body
    })
    _AttributeMap.update({
        __name.name() : __name,
        __type.name() : __type
    })
Namespace.addCategoryObject('typeBinding', 'DeviceGroupChannel', DeviceGroupChannel)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}ChainSelector with content type ELEMENT_ONLY
class ChainSelector (NeuronGroupSelector):
    """A chain of neurons or neuron groups"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'ChainSelector')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 545, 2)
    _ElementMap = NeuronGroupSelector._ElementMap.copy()
    _AttributeMap = NeuronGroupSelector._AttributeMap.copy()
    # Base type is NeuronGroupSelector
    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}neurons uses Python identifier neurons
    __neurons = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'neurons'), 'neurons', '__httpschemas_humanbrainproject_euSP102014BIBI_ChainSelector_httpschemas_humanbrainproject_euSP102014BIBIneurons', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 552, 10), )

    
    neurons = property(__neurons.value, __neurons.set, None, 'Single neuron connections such as single neurons')

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}connectors uses Python identifier connectors
    __connectors = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'connectors'), 'connectors', '__httpschemas_humanbrainproject_euSP102014BIBI_ChainSelector_httpschemas_humanbrainproject_euSP102014BIBIconnectors', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 557, 10), )

    
    connectors = property(__connectors.value, __connectors.set, None, 'Existing groups of neurons')

    _ElementMap.update({
        __neurons.name() : __neurons,
        __connectors.name() : __connectors
    })
    _AttributeMap.update({
        
    })
Namespace.addCategoryObject('typeBinding', 'ChainSelector', ChainSelector)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}MapSelector with content type ELEMENT_ONLY
class MapSelector (NeuronGroupSelector):
    """An indexed mapping of neurons to neuron groups. As index, either a number or a population may be used. In the latter case, the size of the given population is used as count."""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'MapSelector')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 567, 2)
    _ElementMap = NeuronGroupSelector._ElementMap.copy()
    _AttributeMap = NeuronGroupSelector._AttributeMap.copy()
    # Base type is NeuronGroupSelector
    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}count uses Python identifier count
    __count = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'count'), 'count', '__httpschemas_humanbrainproject_euSP102014BIBI_MapSelector_httpschemas_humanbrainproject_euSP102014BIBIcount', False, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 575, 12), )

    
    count = property(__count.value, __count.set, None, 'The number of neural network connections contained in this indexed mapping')

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}source uses Python identifier source
    __source = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'source'), 'source', '__httpschemas_humanbrainproject_euSP102014BIBI_MapSelector_httpschemas_humanbrainproject_euSP102014BIBIsource', False, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 580, 12), )

    
    source = property(__source.value, __source.set, None, 'The source population. If possibility is used, the indexed group consists of one neuron selection per neuron in the source group')

    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}pattern uses Python identifier pattern
    __pattern = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'pattern'), 'pattern', '__httpschemas_humanbrainproject_euSP102014BIBI_MapSelector_httpschemas_humanbrainproject_euSP102014BIBIpattern', False, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 586, 10), )

    
    pattern = property(__pattern.value, __pattern.set, None, 'The pattern that shall be used to select neurons')

    _ElementMap.update({
        __count.name() : __count,
        __source.name() : __source,
        __pattern.name() : __pattern
    })
    _AttributeMap.update({
        
    })
Namespace.addCategoryObject('typeBinding', 'MapSelector', MapSelector)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Index with content type EMPTY
class Index (NeuronSelector):
    """Selection of exactly one neuron using an index of a base population"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'Index')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 607, 2)
    _ElementMap = NeuronSelector._ElementMap.copy()
    _AttributeMap = NeuronSelector._AttributeMap.copy()
    # Base type is NeuronSelector
    
    # Attribute population inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}NeuronSelector
    
    # Attribute index uses Python identifier index
    __index = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'index'), 'index', '__httpschemas_humanbrainproject_euSP102014BIBI_Index_index', pyxb.binding.datatypes.nonNegativeInteger, required=True)
    __index._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 613, 8)
    __index._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 613, 8)
    
    index = property(__index.value, __index.set, None, 'The index of the selected neuron within its population')

    _ElementMap.update({
        
    })
    _AttributeMap.update({
        __index.name() : __index
    })
Namespace.addCategoryObject('typeBinding', 'Index', Index)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}MultiNeuronSelector with content type EMPTY
class MultiNeuronSelector (NeuronSelector):
    """
        The abstract base class of selections of multiple neurons
      """
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
    _Abstract = True
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'MultiNeuronSelector')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 622, 2)
    _ElementMap = NeuronSelector._ElementMap.copy()
    _AttributeMap = NeuronSelector._AttributeMap.copy()
    # Base type is NeuronSelector
    
    # Attribute population inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}NeuronSelector
    _ElementMap.update({
        
    })
    _AttributeMap.update({
        
    })
Namespace.addCategoryObject('typeBinding', 'MultiNeuronSelector', MultiNeuronSelector)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}IndexTemplate with content type EMPTY
class IndexTemplate (NeuronSelectorTemplate):
    """A template for an index-based neuron selection"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'IndexTemplate')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 705, 2)
    _ElementMap = NeuronSelectorTemplate._ElementMap.copy()
    _AttributeMap = NeuronSelectorTemplate._AttributeMap.copy()
    # Base type is NeuronSelectorTemplate
    
    # Attribute index uses Python identifier index
    __index = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'index'), 'index', '__httpschemas_humanbrainproject_euSP102014BIBI_IndexTemplate_index', TemplatePattern, required=True)
    __index._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 711, 8)
    __index._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 711, 8)
    
    index = property(__index.value, __index.set, None, 'The template for the index to access the neurons')

    _ElementMap.update({
        
    })
    _AttributeMap.update({
        __index.name() : __index
    })
Namespace.addCategoryObject('typeBinding', 'IndexTemplate', IndexTemplate)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}RangeTemplate with content type EMPTY
class RangeTemplate (NeuronSelectorTemplate):
    """A template for the range-based neuron selection"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'RangeTemplate')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 720, 2)
    _ElementMap = NeuronSelectorTemplate._ElementMap.copy()
    _AttributeMap = NeuronSelectorTemplate._AttributeMap.copy()
    # Base type is NeuronSelectorTemplate
    
    # Attribute from uses Python identifier from_
    __from = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'from'), 'from_', '__httpschemas_humanbrainproject_euSP102014BIBI_RangeTemplate_from', TemplatePattern, required=True)
    __from._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 726, 8)
    __from._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 726, 8)
    
    from_ = property(__from.value, __from.set, None, 'A template for the start index of the selected range')

    
    # Attribute to uses Python identifier to
    __to = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'to'), 'to', '__httpschemas_humanbrainproject_euSP102014BIBI_RangeTemplate_to', TemplatePattern, required=True)
    __to._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 731, 8)
    __to._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 731, 8)
    
    to = property(__to.value, __to.set, None, 'A template for the end index of the selected range')

    
    # Attribute step uses Python identifier step
    __step = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'step'), 'step', '__httpschemas_humanbrainproject_euSP102014BIBI_RangeTemplate_step', TemplatePattern)
    __step._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 736, 8)
    __step._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 736, 8)
    
    step = property(__step.value, __step.set, None, 'A template for the step of the selected range')

    _ElementMap.update({
        
    })
    _AttributeMap.update({
        __from.name() : __from,
        __to.name() : __to,
        __step.name() : __step
    })
Namespace.addCategoryObject('typeBinding', 'RangeTemplate', RangeTemplate)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}ListTemplate with content type ELEMENT_ONLY
class ListTemplate (NeuronSelectorTemplate):
    """A template for a list-based neuron selection"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'ListTemplate')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 745, 2)
    _ElementMap = NeuronSelectorTemplate._ElementMap.copy()
    _AttributeMap = NeuronSelectorTemplate._AttributeMap.copy()
    # Base type is NeuronSelectorTemplate
    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}element uses Python identifier element
    __element = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'element'), 'element', '__httpschemas_humanbrainproject_euSP102014BIBI_ListTemplate_httpschemas_humanbrainproject_euSP102014BIBIelement', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 752, 10), )

    
    element = property(__element.value, __element.set, None, 'Templates for the indices of selected neurons')

    _ElementMap.update({
        __element.name() : __element
    })
    _AttributeMap.update({
        
    })
Namespace.addCategoryObject('typeBinding', 'ListTemplate', ListTemplate)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}TopicChannel with content type ELEMENT_ONLY
class TopicChannel (pyxb.binding.basis.complexTypeDefinition):
    """A connection of a transfer function to a robot control message topic"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'TopicChannel')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 815, 2)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}body uses Python identifier body
    __body = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'body'), 'body', '__httpschemas_humanbrainproject_euSP102014BIBI_TopicChannel_httpschemas_humanbrainproject_euSP102014BIBIbody', False, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 820, 6), )

    
    body = property(__body.value, __body.set, None, 'The value that should be sent to the robot control topic. If this element is present, then the channel is published to. Otherwise, the channel subscribes to the selected topic.')

    
    # Attribute name uses Python identifier name
    __name = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'name'), 'name', '__httpschemas_humanbrainproject_euSP102014BIBI_TopicChannel_name', pyxb.binding.datatypes.string, required=True)
    __name._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 826, 4)
    __name._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 826, 4)
    
    name = property(__name.value, __name.set, None, 'The name of the robot topic channel')

    
    # Attribute topic uses Python identifier topic
    __topic = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'topic'), 'topic', '__httpschemas_humanbrainproject_euSP102014BIBI_TopicChannel_topic', RobotTopicAddress, required=True)
    __topic._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 831, 4)
    __topic._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 831, 4)
    
    topic = property(__topic.value, __topic.set, None, "The actual topic address, for example '/husky/cmd_vel'")

    
    # Attribute type uses Python identifier type
    __type = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'type'), 'type', '__httpschemas_humanbrainproject_euSP102014BIBI_TopicChannel_type', pyxb.binding.datatypes.string, required=True)
    __type._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 836, 4)
    __type._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 836, 4)
    
    type = property(__type.value, __type.set, None, 'The type of the topic')

    _ElementMap.update({
        __body.name() : __body
    })
    _AttributeMap.update({
        __name.name() : __name,
        __topic.name() : __topic,
        __type.name() : __type
    })
Namespace.addCategoryObject('typeBinding', 'TopicChannel', TopicChannel)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Scale with content type ELEMENT_ONLY
class Scale (FlowExpression):
    """The scaling of an element by a constant factor"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'Scale')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 858, 2)
    _ElementMap = FlowExpression._ElementMap.copy()
    _AttributeMap = FlowExpression._AttributeMap.copy()
    # Base type is FlowExpression
    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}inner uses Python identifier inner
    __inner = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'inner'), 'inner', '__httpschemas_humanbrainproject_euSP102014BIBI_Scale_httpschemas_humanbrainproject_euSP102014BIBIinner', False, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 865, 10), )

    
    inner = property(__inner.value, __inner.set, None, 'The inner flow expression')

    
    # Attribute factor uses Python identifier factor
    __factor = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'factor'), 'factor', '__httpschemas_humanbrainproject_euSP102014BIBI_Scale_factor', pyxb.binding.datatypes.double, required=True)
    __factor._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 871, 8)
    __factor._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 871, 8)
    
    factor = property(__factor.value, __factor.set, None, 'The factor by which the inner expression should be scaled')

    _ElementMap.update({
        __inner.name() : __inner
    })
    _AttributeMap.update({
        __factor.name() : __factor
    })
Namespace.addCategoryObject('typeBinding', 'Scale', Scale)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Call with content type ELEMENT_ONLY
class Call (FlowExpression):
    """A call to a static method"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'Call')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 880, 2)
    _ElementMap = FlowExpression._ElementMap.copy()
    _AttributeMap = FlowExpression._AttributeMap.copy()
    # Base type is FlowExpression
    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}argument uses Python identifier argument
    __argument = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'argument'), 'argument', '__httpschemas_humanbrainproject_euSP102014BIBI_Call_httpschemas_humanbrainproject_euSP102014BIBIargument', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 887, 10), )

    
    argument = property(__argument.value, __argument.set, None, 'Named arguments that are passed to the selected method')

    
    # Attribute type uses Python identifier type
    __type = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'type'), 'type', '__httpschemas_humanbrainproject_euSP102014BIBI_Call_type', pyxb.binding.datatypes.string, required=True)
    __type._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 893, 8)
    __type._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 893, 8)
    
    type = property(__type.value, __type.set, None, 'A reference to the static method. This is specified as a full path of a Python function, including both the path of the module and the name of the function. For this to work, the function must be static, i.e. a global function on that module or a static class function.')

    _ElementMap.update({
        __argument.name() : __argument
    })
    _AttributeMap.update({
        __type.name() : __type
    })
Namespace.addCategoryObject('typeBinding', 'Call', Call)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Operator with content type ELEMENT_ONLY
class Operator (FlowExpression):
    """The abstract base class for an operator call based on a flow expression"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = True
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'Operator')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 902, 2)
    _ElementMap = FlowExpression._ElementMap.copy()
    _AttributeMap = FlowExpression._AttributeMap.copy()
    # Base type is FlowExpression
    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}operand uses Python identifier operand
    __operand = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'operand'), 'operand', '__httpschemas_humanbrainproject_euSP102014BIBI_Operator_httpschemas_humanbrainproject_euSP102014BIBIoperand', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 909, 10), )

    
    operand = property(__operand.value, __operand.set, None, 'The arguments of the operator expression')

    _ElementMap.update({
        __operand.name() : __operand
    })
    _AttributeMap.update({
        
    })
Namespace.addCategoryObject('typeBinding', 'Operator', Operator)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}SimulationStep with content type EMPTY
class SimulationStep (FlowExpression):
    """A reference to the simulation step"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'SimulationStep')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 1025, 2)
    _ElementMap = FlowExpression._ElementMap.copy()
    _AttributeMap = FlowExpression._AttributeMap.copy()
    # Base type is FlowExpression
    _ElementMap.update({
        
    })
    _AttributeMap.update({
        
    })
Namespace.addCategoryObject('typeBinding', 'SimulationStep', SimulationStep)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}ArgumentReference with content type EMPTY
class ArgumentReference (FlowExpression):
    """A reference to an argument, either a device or a local variable"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'ArgumentReference')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 1034, 2)
    _ElementMap = FlowExpression._ElementMap.copy()
    _AttributeMap = FlowExpression._AttributeMap.copy()
    # Base type is FlowExpression
    
    # Attribute name uses Python identifier name
    __name = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'name'), 'name', '__httpschemas_humanbrainproject_euSP102014BIBI_ArgumentReference_name', pyxb.binding.datatypes.string, required=True)
    __name._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 1040, 8)
    __name._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 1040, 8)
    
    name = property(__name.value, __name.set, None, 'The name of the device or local variable')

    
    # Attribute property uses Python identifier property_
    __property = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'property'), 'property_', '__httpschemas_humanbrainproject_euSP102014BIBI_ArgumentReference_property', pyxb.binding.datatypes.string)
    __property._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 1045, 8)
    __property._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 1045, 8)
    
    property_ = property(__property.value, __property.set, None, 'If specified, only a property of the local variable is referenced. Otherwise, the value itself (or the default property of a device) is selected.')

    _ElementMap.update({
        
    })
    _AttributeMap.update({
        __name.name() : __name,
        __property.name() : __property
    })
Namespace.addCategoryObject('typeBinding', 'ArgumentReference', ArgumentReference)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Constant with content type EMPTY
class Constant (FlowExpression):
    """A constant as a flow element"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'Constant')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 1054, 2)
    _ElementMap = FlowExpression._ElementMap.copy()
    _AttributeMap = FlowExpression._AttributeMap.copy()
    # Base type is FlowExpression
    
    # Attribute value uses Python identifier value_
    __value = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'value'), 'value_', '__httpschemas_humanbrainproject_euSP102014BIBI_Constant_value', pyxb.binding.datatypes.double, required=True)
    __value._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 1060, 8)
    __value._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 1060, 8)
    
    value_ = property(__value.value, __value.set, None, 'The value for this constant')

    _ElementMap.update({
        
    })
    _AttributeMap.update({
        __value.name() : __value
    })
Namespace.addCategoryObject('typeBinding', 'Constant', Constant)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}ConstantString with content type EMPTY
class ConstantString (FlowExpression):
    """A constant string"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'ConstantString')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 1069, 2)
    _ElementMap = FlowExpression._ElementMap.copy()
    _AttributeMap = FlowExpression._AttributeMap.copy()
    # Base type is FlowExpression
    
    # Attribute value uses Python identifier value_
    __value = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'value'), 'value_', '__httpschemas_humanbrainproject_euSP102014BIBI_ConstantString_value', pyxb.binding.datatypes.string, required=True)
    __value._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 1075, 8)
    __value._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 1075, 8)
    
    value_ = property(__value.value, __value.set, None, 'The value of this string constant')

    _ElementMap.update({
        
    })
    _AttributeMap.update({
        __value.name() : __value
    })
Namespace.addCategoryObject('typeBinding', 'ConstantString', ConstantString)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}ConfFile with content type EMPTY
class ConfFile (pyxb.binding.basis.complexTypeDefinition):
    """This type denotes an additional configuration entry that consists of a file and a purpose."""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'ConfFile')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 156, 2)
    _ElementMap = {}
    _AttributeMap = {}
    # Base type is pyxb.binding.datatypes.anyType
    
    # Attribute src uses Python identifier src
    __src = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'src'), 'src', '__httpschemas_humanbrainproject_euSP102014BIBI_ConfFile_src', pyxb.binding.datatypes.string, required=True)
    __src._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 160, 4)
    __src._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 160, 4)
    
    src = property(__src.value, __src.set, None, 'The source of a configuration entry is a path to a file that contains the necessary information. The path is relative to the BIBI model.')

    
    # Attribute type uses Python identifier type
    __type = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'type'), 'type', '__httpschemas_humanbrainproject_euSP102014BIBI_ConfFile_type', ConfType, required=True)
    __type._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 165, 4)
    __type._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 165, 4)
    
    type = property(__type.value, __type.set, None, 'The type of a configuration entry denotes the purpose how this entry is used. This is used to decouple the purpose of a configuration entry from the file name.')

    _ElementMap.update({
        
    })
    _AttributeMap.update({
        __src.name() : __src,
        __type.name() : __type
    })
Namespace.addCategoryObject('typeBinding', 'ConfFile', ConfFile)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Robot2Neuron with content type ELEMENT_ONLY
class Robot2Neuron (BIBITransferFunction):
    """A Robot2Neuron transfer function is a transfer function whose primary purpose is to translate information coming from robot sensors, transform it and push them into neural networks. """
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'Robot2Neuron')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 256, 2)
    _ElementMap = BIBITransferFunction._ElementMap.copy()
    _AttributeMap = BIBITransferFunction._AttributeMap.copy()
    # Base type is BIBITransferFunction
    
    # Element local ({http://schemas.humanbrainproject.eu/SP10/2014/BIBI}local) inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}BIBITransferFunction
    
    # Element device ({http://schemas.humanbrainproject.eu/SP10/2014/BIBI}device) inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}BIBITransferFunction
    
    # Element deviceGroup ({http://schemas.humanbrainproject.eu/SP10/2014/BIBI}deviceGroup) inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}BIBITransferFunction
    
    # Element topic ({http://schemas.humanbrainproject.eu/SP10/2014/BIBI}topic) inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}BIBITransferFunction
    
    # Attribute name inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}BIBITransferFunction
    _ElementMap.update({
        
    })
    _AttributeMap.update({
        
    })
Namespace.addCategoryObject('typeBinding', 'Robot2Neuron', Robot2Neuron)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Neuron2Monitor with content type ELEMENT_ONLY
class Neuron2Monitor (BIBITransferFunction):
    """A NeuronMonitor is a special class of transfer functions that monitors neural network populations. Connections to robot control topics or device groups are not allowed."""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'Neuron2Monitor')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 267, 2)
    _ElementMap = BIBITransferFunction._ElementMap.copy()
    _AttributeMap = BIBITransferFunction._AttributeMap.copy()
    # Base type is BIBITransferFunction
    
    # Element local ({http://schemas.humanbrainproject.eu/SP10/2014/BIBI}local) inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}BIBITransferFunction
    
    # Element device ({http://schemas.humanbrainproject.eu/SP10/2014/BIBI}device) inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}BIBITransferFunction
    
    # Attribute name is restricted from parent
    
    # Attribute name uses Python identifier name
    __name = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'name'), 'name', '__httpschemas_humanbrainproject_euSP102014BIBI_BIBITransferFunction_name', pyxb.binding.datatypes.string, required=True)
    __name._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 279, 8)
    __name._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 279, 8)
    
    name = property(__name.value, __name.set, None, None)

    _ElementMap.update({
        
    })
    _AttributeMap.update({
        __name.name() : __name
    })
Namespace.addCategoryObject('typeBinding', 'Neuron2Monitor', Neuron2Monitor)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Neuron2Robot with content type ELEMENT_ONLY
class Neuron2Robot (BIBITransferFunction):
    """A Neuron2Robot transfer function is a transfer function whose primary purpose is to extract information from the neural network and use this information to control the robot using robot control messages"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'Neuron2Robot')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 284, 2)
    _ElementMap = BIBITransferFunction._ElementMap.copy()
    _AttributeMap = BIBITransferFunction._AttributeMap.copy()
    # Base type is BIBITransferFunction
    
    # Element local ({http://schemas.humanbrainproject.eu/SP10/2014/BIBI}local) inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}BIBITransferFunction
    
    # Element device ({http://schemas.humanbrainproject.eu/SP10/2014/BIBI}device) inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}BIBITransferFunction
    
    # Element deviceGroup ({http://schemas.humanbrainproject.eu/SP10/2014/BIBI}deviceGroup) inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}BIBITransferFunction
    
    # Element topic ({http://schemas.humanbrainproject.eu/SP10/2014/BIBI}topic) inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}BIBITransferFunction
    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}returnValue uses Python identifier returnValue
    __returnValue = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'returnValue'), 'returnValue', '__httpschemas_humanbrainproject_euSP102014BIBI_Neuron2Robot_httpschemas_humanbrainproject_euSP102014BIBIreturnValue', False, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 291, 10), )

    
    returnValue = property(__returnValue.value, __returnValue.set, None, 'The return value topic channel of a Neuron2Robot transfer function is the channel to which control messages the return value of the Python function are sent')

    
    # Attribute name inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}BIBITransferFunction
    _ElementMap.update({
        __returnValue.name() : __returnValue
    })
    _AttributeMap.update({
        
    })
Namespace.addCategoryObject('typeBinding', 'Neuron2Robot', Neuron2Robot)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Range with content type EMPTY
class Range (MultiNeuronSelector):
    """Selection of a range of neurons from an existing population"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'Range')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 633, 2)
    _ElementMap = MultiNeuronSelector._ElementMap.copy()
    _AttributeMap = MultiNeuronSelector._AttributeMap.copy()
    # Base type is MultiNeuronSelector
    
    # Attribute population inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}NeuronSelector
    
    # Attribute from uses Python identifier from_
    __from = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'from'), 'from_', '__httpschemas_humanbrainproject_euSP102014BIBI_Range_from', pyxb.binding.datatypes.nonNegativeInteger, required=True)
    __from._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 639, 8)
    __from._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 639, 8)
    
    from_ = property(__from.value, __from.set, None, 'The starting index from which neurons are selected')

    
    # Attribute to uses Python identifier to
    __to = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'to'), 'to', '__httpschemas_humanbrainproject_euSP102014BIBI_Range_to', pyxb.binding.datatypes.nonNegativeInteger, required=True)
    __to._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 644, 8)
    __to._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 644, 8)
    
    to = property(__to.value, __to.set, None, 'The stop index to which neurons are selected')

    
    # Attribute step uses Python identifier step
    __step = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'step'), 'step', '__httpschemas_humanbrainproject_euSP102014BIBI_Range_step', pyxb.binding.datatypes.positiveInteger)
    __step._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 649, 8)
    __step._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 649, 8)
    
    step = property(__step.value, __step.set, None, 'The step of the selection')

    _ElementMap.update({
        
    })
    _AttributeMap.update({
        __from.name() : __from,
        __to.name() : __to,
        __step.name() : __step
    })
Namespace.addCategoryObject('typeBinding', 'Range', Range)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}List with content type ELEMENT_ONLY
class List (MultiNeuronSelector):
    """Selection of a list of neurons using their indices"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'List')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 658, 2)
    _ElementMap = MultiNeuronSelector._ElementMap.copy()
    _AttributeMap = MultiNeuronSelector._AttributeMap.copy()
    # Base type is MultiNeuronSelector
    
    # Element {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}element uses Python identifier element
    __element = pyxb.binding.content.ElementDeclaration(pyxb.namespace.ExpandedName(Namespace, 'element'), 'element', '__httpschemas_humanbrainproject_euSP102014BIBI_List_httpschemas_humanbrainproject_euSP102014BIBIelement', True, pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 665, 10), )

    
    element = property(__element.value, __element.set, None, 'The indices of selected neurons')

    
    # Attribute population inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}NeuronSelector
    _ElementMap.update({
        __element.name() : __element
    })
    _AttributeMap.update({
        
    })
Namespace.addCategoryObject('typeBinding', 'List', List)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Population with content type EMPTY
class Population (MultiNeuronSelector):
    """Selection of an entire population of neurons"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_EMPTY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'Population')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 675, 2)
    _ElementMap = MultiNeuronSelector._ElementMap.copy()
    _AttributeMap = MultiNeuronSelector._AttributeMap.copy()
    # Base type is MultiNeuronSelector
    
    # Attribute population inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}NeuronSelector
    
    # Attribute count uses Python identifier count
    __count = pyxb.binding.content.AttributeUse(pyxb.namespace.ExpandedName(None, 'count'), 'count', '__httpschemas_humanbrainproject_euSP102014BIBI_Population_count', pyxb.binding.datatypes.positiveInteger, required=True)
    __count._DeclarationLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 681, 8)
    __count._UseLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 681, 8)
    
    count = property(__count.value, __count.set, None, 'The size of the selected population. This is necessary for validation purposes where the neural network is not available.')

    _ElementMap.update({
        
    })
    _AttributeMap.update({
        __count.name() : __count
    })
Namespace.addCategoryObject('typeBinding', 'Population', Population)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Add with content type ELEMENT_ONLY
class Add (Operator):
    """The sum of all operands"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'Add')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 919, 2)
    _ElementMap = Operator._ElementMap.copy()
    _AttributeMap = Operator._AttributeMap.copy()
    # Base type is Operator
    
    # Element operand ({http://schemas.humanbrainproject.eu/SP10/2014/BIBI}operand) inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Operator
    _ElementMap.update({
        
    })
    _AttributeMap.update({
        
    })
Namespace.addCategoryObject('typeBinding', 'Add', Add)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Subtract with content type ELEMENT_ONLY
class Subtract (Operator):
    """The difference between two operands"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'Subtract')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 928, 2)
    _ElementMap = Operator._ElementMap.copy()
    _AttributeMap = Operator._AttributeMap.copy()
    # Base type is Operator
    
    # Element operand ({http://schemas.humanbrainproject.eu/SP10/2014/BIBI}operand) inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Operator
    _ElementMap.update({
        
    })
    _AttributeMap.update({
        
    })
Namespace.addCategoryObject('typeBinding', 'Subtract', Subtract)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Multiply with content type ELEMENT_ONLY
class Multiply (Operator):
    """The product of all operands"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'Multiply')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 945, 2)
    _ElementMap = Operator._ElementMap.copy()
    _AttributeMap = Operator._AttributeMap.copy()
    # Base type is Operator
    
    # Element operand ({http://schemas.humanbrainproject.eu/SP10/2014/BIBI}operand) inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Operator
    _ElementMap.update({
        
    })
    _AttributeMap.update({
        
    })
Namespace.addCategoryObject('typeBinding', 'Multiply', Multiply)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Divide with content type ELEMENT_ONLY
class Divide (Operator):
    """The quotient of two operands"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'Divide')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 954, 2)
    _ElementMap = Operator._ElementMap.copy()
    _AttributeMap = Operator._AttributeMap.copy()
    # Base type is Operator
    
    # Element operand ({http://schemas.humanbrainproject.eu/SP10/2014/BIBI}operand) inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Operator
    _ElementMap.update({
        
    })
    _AttributeMap.update({
        
    })
Namespace.addCategoryObject('typeBinding', 'Divide', Divide)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Min with content type ELEMENT_ONLY
class Min (Operator):
    """The minimum of the provided values"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'Min')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 971, 2)
    _ElementMap = Operator._ElementMap.copy()
    _AttributeMap = Operator._AttributeMap.copy()
    # Base type is Operator
    
    # Element operand ({http://schemas.humanbrainproject.eu/SP10/2014/BIBI}operand) inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Operator
    _ElementMap.update({
        
    })
    _AttributeMap.update({
        
    })
Namespace.addCategoryObject('typeBinding', 'Min', Min)


# Complex type {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Max with content type ELEMENT_ONLY
class Max (Operator):
    """The maximum of the provided values"""
    _TypeDefinition = None
    _ContentTypeTag = pyxb.binding.basis.complexTypeDefinition._CT_ELEMENT_ONLY
    _Abstract = False
    _ExpandedName = pyxb.namespace.ExpandedName(Namespace, 'Max')
    _XSDLocation = pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 980, 2)
    _ElementMap = Operator._ElementMap.copy()
    _AttributeMap = Operator._AttributeMap.copy()
    # Base type is Operator
    
    # Element operand ({http://schemas.humanbrainproject.eu/SP10/2014/BIBI}operand) inherited from {http://schemas.humanbrainproject.eu/SP10/2014/BIBI}Operator
    _ElementMap.update({
        
    })
    _AttributeMap.update({
        
    })
Namespace.addCategoryObject('typeBinding', 'Max', Max)


bibi = pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'bibi'), BIBIConfiguration, location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 8, 2))
Namespace.addCategoryObject('elementBinding', bibi.name().localName(), bibi)



BIBIConfiguration._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'timestep'), TimeStep, scope=BIBIConfiguration, documentation='If specified, the CLE uses a different timestep than the default timestep of 20ms. The timestep is specified in milliseconds and depicts the time between two successive loops of the CLE in simulation time.', location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 14, 6)))

BIBIConfiguration._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'brainModel'), BrainModel, scope=BIBIConfiguration, documentation='The brain model depicts a path to the neural network model.', location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 19, 6)))

BIBIConfiguration._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'bodyModel'), SDFFilename, scope=BIBIConfiguration, documentation='The path to the robot model that should be used. This can either be a path to an SDF model or a path to a zip file containing all required assets for a robot. This zip file must have a file model.sdf at the root of the archive.', location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 24, 6)))

BIBIConfiguration._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'mode'), SimulationMode, scope=BIBIConfiguration, documentation='The simulation mode. This determines the choice of the neural network simulator.', location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 29, 6)))

BIBIConfiguration._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'extRobotController'), ScriptFilename, scope=BIBIConfiguration, documentation='A path to an external robot controller. If specified, the robot controller is started when the simulation begins and stopped when the simulation is over. Therefore, the path must be a path to a shell script that offers a function start and a function stop.', location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 34, 6)))

BIBIConfiguration._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'configuration'), ConfFile, scope=BIBIConfiguration, documentation='The configuration entries of an experiment depict additional files required for the simulation of experiments using this BIBI configuration.', location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 39, 6)))

BIBIConfiguration._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'connectors'), NeuronConnector, scope=BIBIConfiguration, documentation='A list of connectors. This can be useful when specifying transfer functions ', location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 44, 6)))

BIBIConfiguration._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'synapseDynamics'), SynapseDynamics, scope=BIBIConfiguration, documentation='A list of synapse dynamics. Such a synapse dynamic can be referenced later on in neural network devices.', location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 49, 6)))

BIBIConfiguration._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'transferFunction'), TransferFunction, scope=BIBIConfiguration, documentation='The transfer functions that are used to couple a neural network to robot', location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 54, 6)))

def _BuildAutomaton ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton
    del _BuildAutomaton
    import pyxb.utils.fac as fac

    counters = set()
    cc_0 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 14, 6))
    counters.add(cc_0)
    cc_1 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 29, 6))
    counters.add(cc_1)
    cc_2 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 34, 6))
    counters.add(cc_2)
    cc_3 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 39, 6))
    counters.add(cc_3)
    cc_4 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 44, 6))
    counters.add(cc_4)
    cc_5 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 49, 6))
    counters.add(cc_5)
    cc_6 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 54, 6))
    counters.add(cc_6)
    states = []
    final_update = None
    symbol = pyxb.binding.content.ElementUse(BIBIConfiguration._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'timestep')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 14, 6))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    final_update = None
    symbol = pyxb.binding.content.ElementUse(BIBIConfiguration._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'brainModel')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 19, 6))
    st_1 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_1)
    final_update = set()
    symbol = pyxb.binding.content.ElementUse(BIBIConfiguration._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'bodyModel')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 24, 6))
    st_2 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_2)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_1, False))
    symbol = pyxb.binding.content.ElementUse(BIBIConfiguration._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'mode')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 29, 6))
    st_3 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_3)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_2, False))
    symbol = pyxb.binding.content.ElementUse(BIBIConfiguration._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'extRobotController')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 34, 6))
    st_4 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_4)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_3, False))
    symbol = pyxb.binding.content.ElementUse(BIBIConfiguration._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'configuration')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 39, 6))
    st_5 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_5)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_4, False))
    symbol = pyxb.binding.content.ElementUse(BIBIConfiguration._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'connectors')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 44, 6))
    st_6 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_6)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_5, False))
    symbol = pyxb.binding.content.ElementUse(BIBIConfiguration._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'synapseDynamics')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 49, 6))
    st_7 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_7)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_6, False))
    symbol = pyxb.binding.content.ElementUse(BIBIConfiguration._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'transferFunction')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 54, 6))
    st_8 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_8)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, False) ]))
    st_0._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_2, [
         ]))
    st_1._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_3, [
         ]))
    transitions.append(fac.Transition(st_4, [
         ]))
    transitions.append(fac.Transition(st_5, [
         ]))
    transitions.append(fac.Transition(st_6, [
         ]))
    transitions.append(fac.Transition(st_7, [
         ]))
    transitions.append(fac.Transition(st_8, [
         ]))
    st_2._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_1, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_1, False) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_1, False) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_1, False) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_1, False) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_1, False) ]))
    st_3._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_2, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_2, False) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_2, False) ]))
    st_4._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_3, True) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_3, False) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_3, False) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_3, False) ]))
    st_5._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_4, True) ]))
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_4, False) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_4, False) ]))
    st_6._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_7, [
        fac.UpdateInstruction(cc_5, True) ]))
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_5, False) ]))
    st_7._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_8, [
        fac.UpdateInstruction(cc_6, True) ]))
    st_8._set_transitionSet(transitions)
    return fac.Automaton(states, counters, False, containing_state=None)
BIBIConfiguration._Automaton = _BuildAutomaton()




BrainModel._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'file'), BrainFilename, scope=BrainModel, documentation='A path to the neural network file.', location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 99, 6)))

BrainModel._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'populations'), MultiNeuronSelector, scope=BrainModel, documentation="The populations in this field are the explicitly defined populations. Each of this population is defined as a view of an assumed 'circuit' population.", location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 104, 6)))

def _BuildAutomaton_ ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_
    del _BuildAutomaton_
    import pyxb.utils.fac as fac

    counters = set()
    cc_0 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 104, 6))
    counters.add(cc_0)
    states = []
    final_update = set()
    symbol = pyxb.binding.content.ElementUse(BrainModel._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'file')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 99, 6))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(BrainModel._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'populations')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 104, 6))
    st_1 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_1)
    transitions = []
    transitions.append(fac.Transition(st_1, [
         ]))
    st_0._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_1._set_transitionSet(transitions)
    return fac.Automaton(states, counters, False, containing_state=None)
BrainModel._Automaton = _BuildAutomaton_()




Argument._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'value'), FlowExpression, scope=Argument, documentation='The value passed for this argument', location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 994, 6)))

def _BuildAutomaton_2 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_2
    del _BuildAutomaton_2
    import pyxb.utils.fac as fac

    counters = set()
    states = []
    final_update = set()
    symbol = pyxb.binding.content.ElementUse(Argument._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'value')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 994, 6))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    transitions = []
    st_0._set_transitionSet(transitions)
    return fac.Automaton(states, counters, False, containing_state=None)
Argument._Automaton = _BuildAutomaton_2()




Local._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'body'), FlowExpression, scope=Local, documentation='The initial value for this local variable', location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 1012, 6)))

def _BuildAutomaton_3 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_3
    del _BuildAutomaton_3
    import pyxb.utils.fac as fac

    counters = set()
    states = []
    final_update = set()
    symbol = pyxb.binding.content.ElementUse(Local._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'body')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 1012, 6))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    transitions = []
    st_0._set_transitionSet(transitions)
    return fac.Automaton(states, counters, False, containing_state=None)
Local._Automaton = _BuildAutomaton_3()




def _BuildAutomaton_4 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_4
    del _BuildAutomaton_4
    import pyxb.utils.fac as fac

    counters = set()
    cc_0 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 203, 10))
    counters.add(cc_0)
    states = []
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.WildcardUse(pyxb.binding.content.Wildcard(process_contents=pyxb.binding.content.Wildcard.PC_skip, namespace_constraint=(pyxb.binding.content.Wildcard.NC_not, 'http://schemas.humanbrainproject.eu/SP10/2014/BIBI')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 203, 10))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_0._set_transitionSet(transitions)
    return fac.Automaton(states, counters, True, containing_state=None)
PythonTransferFunction._Automaton = _BuildAutomaton_4()




BIBITransferFunction._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'local'), Local, scope=BIBITransferFunction, documentation='This denotes the local variables of this transfer function.', location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 224, 10)))

BIBITransferFunction._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'device'), DeviceChannel, scope=BIBITransferFunction, documentation='This denotes device channels, connections of the transfer function to the neural network using exactly one device.', location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 230, 12)))

BIBITransferFunction._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'deviceGroup'), DeviceGroupChannel, scope=BIBITransferFunction, documentation='This denotes the device group channels, connections of transfer functions to the neural network using a one-dimensional array of devices.', location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 235, 12)))

BIBITransferFunction._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'topic'), TopicChannel, scope=BIBITransferFunction, documentation='This denotes the connections of the transfer function to robot control channels.', location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 241, 10)))

def _BuildAutomaton_5 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_5
    del _BuildAutomaton_5
    import pyxb.utils.fac as fac

    counters = set()
    cc_0 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 224, 10))
    counters.add(cc_0)
    cc_1 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 229, 10))
    counters.add(cc_1)
    cc_2 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 241, 10))
    counters.add(cc_2)
    states = []
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(BIBITransferFunction._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'local')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 224, 10))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_1, False))
    symbol = pyxb.binding.content.ElementUse(BIBITransferFunction._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'device')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 230, 12))
    st_1 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_1)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_1, False))
    symbol = pyxb.binding.content.ElementUse(BIBITransferFunction._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'deviceGroup')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 235, 12))
    st_2 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_2)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_2, False))
    symbol = pyxb.binding.content.ElementUse(BIBITransferFunction._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'topic')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 241, 10))
    st_3 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_3)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, False) ]))
    st_0._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_1, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_1, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_1, False) ]))
    st_1._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_1, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_1, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_1, False) ]))
    st_2._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_2, True) ]))
    st_3._set_transitionSet(transitions)
    return fac.Automaton(states, counters, True, containing_state=None)
BIBITransferFunction._Automaton = _BuildAutomaton_5()




DeviceChannel._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'neurons'), NeuronSelector, scope=DeviceChannel, documentation='This specifies the neurons that should be connected to this neural connector device', location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 430, 6)))

DeviceChannel._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'connector'), NeuronConnector, scope=DeviceChannel, documentation='Additional information on the connection to the neurons', location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 436, 8)))

DeviceChannel._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'connectorRef'), NeuronConnectorRef, scope=DeviceChannel, documentation='A reference to a reusable connector', location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 441, 8)))

DeviceChannel._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'synapseDynamics'), SynapseDynamics, scope=DeviceChannel, documentation='Additional information on the dynamics of the connection of this device to the neural network', location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 448, 8)))

DeviceChannel._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'synapseDynamicsRef'), SynapseDynamicsRef, scope=DeviceChannel, documentation='A reference to a reusable synapse dynamics', location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 453, 8)))

DeviceChannel._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'target'), NeuronTarget, scope=DeviceChannel, documentation='The target of this connection. This configuration is useful in particular for spike source devices such as Poisson generators. By default, these devices are excitatory but they can be configured to inhibit connected neurons.', location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 459, 6)))

DeviceChannel._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'body'), FlowExpression, scope=DeviceChannel, documentation='This element is only meaningful for spike sources. It depicts the value to which the device should be configured.', location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 464, 6)))

def _BuildAutomaton_6 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_6
    del _BuildAutomaton_6
    import pyxb.utils.fac as fac

    counters = set()
    cc_0 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 435, 6))
    counters.add(cc_0)
    cc_1 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 447, 6))
    counters.add(cc_1)
    cc_2 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 459, 6))
    counters.add(cc_2)
    cc_3 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 464, 6))
    counters.add(cc_3)
    states = []
    final_update = set()
    symbol = pyxb.binding.content.ElementUse(DeviceChannel._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'neurons')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 430, 6))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(DeviceChannel._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'connector')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 436, 8))
    st_1 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_1)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(DeviceChannel._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'connectorRef')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 441, 8))
    st_2 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_2)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_1, False))
    symbol = pyxb.binding.content.ElementUse(DeviceChannel._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'synapseDynamics')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 448, 8))
    st_3 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_3)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_1, False))
    symbol = pyxb.binding.content.ElementUse(DeviceChannel._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'synapseDynamicsRef')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 453, 8))
    st_4 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_4)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_2, False))
    symbol = pyxb.binding.content.ElementUse(DeviceChannel._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'target')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 459, 6))
    st_5 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_5)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_3, False))
    symbol = pyxb.binding.content.ElementUse(DeviceChannel._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'body')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 464, 6))
    st_6 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_6)
    transitions = []
    transitions.append(fac.Transition(st_1, [
         ]))
    transitions.append(fac.Transition(st_2, [
         ]))
    transitions.append(fac.Transition(st_3, [
         ]))
    transitions.append(fac.Transition(st_4, [
         ]))
    transitions.append(fac.Transition(st_5, [
         ]))
    transitions.append(fac.Transition(st_6, [
         ]))
    st_0._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, False) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, False) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, False) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, False) ]))
    st_1._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, False) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, False) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, False) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, False) ]))
    st_2._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_1, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_1, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_1, False) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_1, False) ]))
    st_3._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_1, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_1, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_1, False) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_1, False) ]))
    st_4._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_2, True) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_2, False) ]))
    st_5._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_3, True) ]))
    st_6._set_transitionSet(transitions)
    return fac.Automaton(states, counters, False, containing_state=None)
DeviceChannel._Automaton = _BuildAutomaton_6()




DeviceGroupChannel._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'neurons'), NeuronGroupSelector, scope=DeviceGroupChannel, documentation='This specifies the neurons that should be connected to this neural connector device', location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 487, 6)))

DeviceGroupChannel._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'connector'), NeuronConnector, scope=DeviceGroupChannel, documentation='Additional information on the connection to the neurons', location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 493, 8)))

DeviceGroupChannel._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'connectorRef'), NeuronConnectorRef, scope=DeviceGroupChannel, documentation='A reference to a reusable connector', location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 498, 8)))

DeviceGroupChannel._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'synapseDynamics'), SynapseDynamics, scope=DeviceGroupChannel, documentation='Additional information on the dynamics of the connection of this device to the neural network', location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 505, 8)))

DeviceGroupChannel._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'synapseDynamicsRef'), SynapseDynamicsRef, scope=DeviceGroupChannel, documentation='A reference to a reusable synapse dynamics', location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 510, 8)))

DeviceGroupChannel._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'target'), NeuronTarget, scope=DeviceGroupChannel, documentation='The target of this connection. This configuration is useful in particular for spike source devices such as Poisson generators. By default, these devices are excitatory but they can be configured to inhibit connected neurons.', location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 516, 6)))

DeviceGroupChannel._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'body'), FlowExpression, scope=DeviceGroupChannel, documentation='This element is only meaningful for spike sources. It depicts the value to which the device should be configured.', location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 521, 6)))

def _BuildAutomaton_7 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_7
    del _BuildAutomaton_7
    import pyxb.utils.fac as fac

    counters = set()
    cc_0 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 492, 6))
    counters.add(cc_0)
    cc_1 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 504, 6))
    counters.add(cc_1)
    cc_2 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 516, 6))
    counters.add(cc_2)
    cc_3 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 521, 6))
    counters.add(cc_3)
    states = []
    final_update = set()
    symbol = pyxb.binding.content.ElementUse(DeviceGroupChannel._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'neurons')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 487, 6))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(DeviceGroupChannel._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'connector')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 493, 8))
    st_1 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_1)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(DeviceGroupChannel._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'connectorRef')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 498, 8))
    st_2 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_2)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_1, False))
    symbol = pyxb.binding.content.ElementUse(DeviceGroupChannel._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'synapseDynamics')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 505, 8))
    st_3 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_3)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_1, False))
    symbol = pyxb.binding.content.ElementUse(DeviceGroupChannel._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'synapseDynamicsRef')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 510, 8))
    st_4 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_4)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_2, False))
    symbol = pyxb.binding.content.ElementUse(DeviceGroupChannel._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'target')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 516, 6))
    st_5 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_5)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_3, False))
    symbol = pyxb.binding.content.ElementUse(DeviceGroupChannel._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'body')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 521, 6))
    st_6 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_6)
    transitions = []
    transitions.append(fac.Transition(st_1, [
         ]))
    transitions.append(fac.Transition(st_2, [
         ]))
    transitions.append(fac.Transition(st_3, [
         ]))
    transitions.append(fac.Transition(st_4, [
         ]))
    transitions.append(fac.Transition(st_5, [
         ]))
    transitions.append(fac.Transition(st_6, [
         ]))
    st_0._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, False) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, False) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, False) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, False) ]))
    st_1._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, False) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, False) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_0, False) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_0, False) ]))
    st_2._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_1, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_1, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_1, False) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_1, False) ]))
    st_3._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_1, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_1, True) ]))
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_1, False) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_1, False) ]))
    st_4._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_5, [
        fac.UpdateInstruction(cc_2, True) ]))
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_2, False) ]))
    st_5._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_6, [
        fac.UpdateInstruction(cc_3, True) ]))
    st_6._set_transitionSet(transitions)
    return fac.Automaton(states, counters, False, containing_state=None)
DeviceGroupChannel._Automaton = _BuildAutomaton_7()




ChainSelector._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'neurons'), NeuronSelector, scope=ChainSelector, documentation='Single neuron connections such as single neurons', location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 552, 10)))

ChainSelector._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'connectors'), NeuronGroupSelector, scope=ChainSelector, documentation='Existing groups of neurons', location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 557, 10)))

def _BuildAutomaton_8 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_8
    del _BuildAutomaton_8
    import pyxb.utils.fac as fac

    counters = set()
    cc_0 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 552, 10))
    counters.add(cc_0)
    cc_1 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 557, 10))
    counters.add(cc_1)
    states = []
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(ChainSelector._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'neurons')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 552, 10))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_1, False))
    symbol = pyxb.binding.content.ElementUse(ChainSelector._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'connectors')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 557, 10))
    st_1 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_1)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, False) ]))
    st_0._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_1, True) ]))
    st_1._set_transitionSet(transitions)
    return fac.Automaton(states, counters, True, containing_state=None)
ChainSelector._Automaton = _BuildAutomaton_8()




MapSelector._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'count'), pyxb.binding.datatypes.positiveInteger, scope=MapSelector, documentation='The number of neural network connections contained in this indexed mapping', location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 575, 12)))

MapSelector._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'source'), MultiNeuronSelector, scope=MapSelector, documentation='The source population. If possibility is used, the indexed group consists of one neuron selection per neuron in the source group', location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 580, 12)))

MapSelector._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'pattern'), NeuronSelectorTemplate, scope=MapSelector, documentation='The pattern that shall be used to select neurons', location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 586, 10)))

def _BuildAutomaton_9 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_9
    del _BuildAutomaton_9
    import pyxb.utils.fac as fac

    counters = set()
    states = []
    final_update = None
    symbol = pyxb.binding.content.ElementUse(MapSelector._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'count')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 575, 12))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    final_update = None
    symbol = pyxb.binding.content.ElementUse(MapSelector._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'source')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 580, 12))
    st_1 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_1)
    final_update = set()
    symbol = pyxb.binding.content.ElementUse(MapSelector._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'pattern')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 586, 10))
    st_2 = fac.State(symbol, is_initial=False, final_update=final_update, is_unordered_catenation=False)
    states.append(st_2)
    transitions = []
    transitions.append(fac.Transition(st_2, [
         ]))
    st_0._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_2, [
         ]))
    st_1._set_transitionSet(transitions)
    transitions = []
    st_2._set_transitionSet(transitions)
    return fac.Automaton(states, counters, False, containing_state=None)
MapSelector._Automaton = _BuildAutomaton_9()




ListTemplate._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'element'), TemplatePattern, scope=ListTemplate, documentation='Templates for the indices of selected neurons', location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 752, 10)))

def _BuildAutomaton_10 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_10
    del _BuildAutomaton_10
    import pyxb.utils.fac as fac

    counters = set()
    states = []
    final_update = set()
    symbol = pyxb.binding.content.ElementUse(ListTemplate._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'element')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 752, 10))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    transitions = []
    transitions.append(fac.Transition(st_0, [
         ]))
    st_0._set_transitionSet(transitions)
    return fac.Automaton(states, counters, False, containing_state=None)
ListTemplate._Automaton = _BuildAutomaton_10()




TopicChannel._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'body'), FlowExpression, scope=TopicChannel, documentation='The value that should be sent to the robot control topic. If this element is present, then the channel is published to. Otherwise, the channel subscribes to the selected topic.', location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 820, 6)))

def _BuildAutomaton_11 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_11
    del _BuildAutomaton_11
    import pyxb.utils.fac as fac

    counters = set()
    cc_0 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 820, 6))
    counters.add(cc_0)
    states = []
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(TopicChannel._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'body')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 820, 6))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_0._set_transitionSet(transitions)
    return fac.Automaton(states, counters, True, containing_state=None)
TopicChannel._Automaton = _BuildAutomaton_11()




Scale._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'inner'), FlowExpression, scope=Scale, documentation='The inner flow expression', location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 865, 10)))

def _BuildAutomaton_12 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_12
    del _BuildAutomaton_12
    import pyxb.utils.fac as fac

    counters = set()
    states = []
    final_update = set()
    symbol = pyxb.binding.content.ElementUse(Scale._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'inner')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 865, 10))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    transitions = []
    st_0._set_transitionSet(transitions)
    return fac.Automaton(states, counters, False, containing_state=None)
Scale._Automaton = _BuildAutomaton_12()




Call._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'argument'), Argument, scope=Call, documentation='Named arguments that are passed to the selected method', location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 887, 10)))

def _BuildAutomaton_13 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_13
    del _BuildAutomaton_13
    import pyxb.utils.fac as fac

    counters = set()
    states = []
    final_update = set()
    symbol = pyxb.binding.content.ElementUse(Call._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'argument')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 887, 10))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    transitions = []
    transitions.append(fac.Transition(st_0, [
         ]))
    st_0._set_transitionSet(transitions)
    return fac.Automaton(states, counters, False, containing_state=None)
Call._Automaton = _BuildAutomaton_13()




Operator._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'operand'), FlowExpression, scope=Operator, documentation='The arguments of the operator expression', location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 909, 10)))

def _BuildAutomaton_14 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_14
    del _BuildAutomaton_14
    import pyxb.utils.fac as fac

    counters = set()
    cc_0 = fac.CounterCondition(min=2, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 909, 10))
    counters.add(cc_0)
    states = []
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(Operator._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'operand')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 909, 10))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_0._set_transitionSet(transitions)
    return fac.Automaton(states, counters, False, containing_state=None)
Operator._Automaton = _BuildAutomaton_14()




def _BuildAutomaton_15 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_15
    del _BuildAutomaton_15
    import pyxb.utils.fac as fac

    counters = set()
    cc_0 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 224, 10))
    counters.add(cc_0)
    cc_1 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 229, 10))
    counters.add(cc_1)
    cc_2 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 241, 10))
    counters.add(cc_2)
    states = []
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(Robot2Neuron._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'local')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 224, 10))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_1, False))
    symbol = pyxb.binding.content.ElementUse(Robot2Neuron._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'device')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 230, 12))
    st_1 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_1)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_1, False))
    symbol = pyxb.binding.content.ElementUse(Robot2Neuron._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'deviceGroup')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 235, 12))
    st_2 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_2)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_2, False))
    symbol = pyxb.binding.content.ElementUse(Robot2Neuron._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'topic')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 241, 10))
    st_3 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_3)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, False) ]))
    st_0._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_1, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_1, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_1, False) ]))
    st_1._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_1, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_1, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_1, False) ]))
    st_2._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_2, True) ]))
    st_3._set_transitionSet(transitions)
    return fac.Automaton(states, counters, True, containing_state=None)
Robot2Neuron._Automaton = _BuildAutomaton_15()




def _BuildAutomaton_16 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_16
    del _BuildAutomaton_16
    import pyxb.utils.fac as fac

    counters = set()
    cc_0 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 274, 10))
    counters.add(cc_0)
    cc_1 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 275, 10))
    counters.add(cc_1)
    states = []
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(Neuron2Monitor._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'local')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 274, 10))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_1, False))
    symbol = pyxb.binding.content.ElementUse(Neuron2Monitor._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'device')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 276, 12))
    st_1 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_1)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, False) ]))
    st_0._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_1, True) ]))
    st_1._set_transitionSet(transitions)
    return fac.Automaton(states, counters, True, containing_state=None)
Neuron2Monitor._Automaton = _BuildAutomaton_16()




Neuron2Robot._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'returnValue'), TopicChannel, scope=Neuron2Robot, documentation='The return value topic channel of a Neuron2Robot transfer function is the channel to which control messages the return value of the Python function are sent', location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 291, 10)))

def _BuildAutomaton_17 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_17
    del _BuildAutomaton_17
    import pyxb.utils.fac as fac

    counters = set()
    cc_0 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 224, 10))
    counters.add(cc_0)
    cc_1 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 229, 10))
    counters.add(cc_1)
    cc_2 = fac.CounterCondition(min=0, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 241, 10))
    counters.add(cc_2)
    cc_3 = fac.CounterCondition(min=0, max=1, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 291, 10))
    counters.add(cc_3)
    states = []
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(Neuron2Robot._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'local')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 224, 10))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_1, False))
    symbol = pyxb.binding.content.ElementUse(Neuron2Robot._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'device')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 230, 12))
    st_1 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_1)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_1, False))
    symbol = pyxb.binding.content.ElementUse(Neuron2Robot._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'deviceGroup')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 235, 12))
    st_2 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_2)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_2, False))
    symbol = pyxb.binding.content.ElementUse(Neuron2Robot._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'topic')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 241, 10))
    st_3 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_3)
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_3, False))
    symbol = pyxb.binding.content.ElementUse(Neuron2Robot._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'returnValue')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 291, 10))
    st_4 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_4)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_0, False) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_0, False) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_0, False) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_0, False) ]))
    st_0._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_1, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_1, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_1, False) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_1, False) ]))
    st_1._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_1, [
        fac.UpdateInstruction(cc_1, True) ]))
    transitions.append(fac.Transition(st_2, [
        fac.UpdateInstruction(cc_1, True) ]))
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_1, False) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_1, False) ]))
    st_2._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_3, [
        fac.UpdateInstruction(cc_2, True) ]))
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_2, False) ]))
    st_3._set_transitionSet(transitions)
    transitions = []
    transitions.append(fac.Transition(st_4, [
        fac.UpdateInstruction(cc_3, True) ]))
    st_4._set_transitionSet(transitions)
    return fac.Automaton(states, counters, True, containing_state=None)
Neuron2Robot._Automaton = _BuildAutomaton_17()




List._AddElement(pyxb.binding.basis.element(pyxb.namespace.ExpandedName(Namespace, 'element'), pyxb.binding.datatypes.nonNegativeInteger, scope=List, documentation='The indices of selected neurons', location=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 665, 10)))

def _BuildAutomaton_18 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_18
    del _BuildAutomaton_18
    import pyxb.utils.fac as fac

    counters = set()
    states = []
    final_update = set()
    symbol = pyxb.binding.content.ElementUse(List._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'element')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 665, 10))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    transitions = []
    transitions.append(fac.Transition(st_0, [
         ]))
    st_0._set_transitionSet(transitions)
    return fac.Automaton(states, counters, False, containing_state=None)
List._Automaton = _BuildAutomaton_18()




def _BuildAutomaton_19 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_19
    del _BuildAutomaton_19
    import pyxb.utils.fac as fac

    counters = set()
    cc_0 = fac.CounterCondition(min=2, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 909, 10))
    counters.add(cc_0)
    states = []
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(Add._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'operand')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 909, 10))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_0._set_transitionSet(transitions)
    return fac.Automaton(states, counters, False, containing_state=None)
Add._Automaton = _BuildAutomaton_19()




def _BuildAutomaton_20 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_20
    del _BuildAutomaton_20
    import pyxb.utils.fac as fac

    counters = set()
    cc_0 = fac.CounterCondition(min=2, max=2, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 935, 10))
    counters.add(cc_0)
    states = []
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(Subtract._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'operand')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 935, 10))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_0._set_transitionSet(transitions)
    return fac.Automaton(states, counters, False, containing_state=None)
Subtract._Automaton = _BuildAutomaton_20()




def _BuildAutomaton_21 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_21
    del _BuildAutomaton_21
    import pyxb.utils.fac as fac

    counters = set()
    cc_0 = fac.CounterCondition(min=2, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 909, 10))
    counters.add(cc_0)
    states = []
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(Multiply._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'operand')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 909, 10))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_0._set_transitionSet(transitions)
    return fac.Automaton(states, counters, False, containing_state=None)
Multiply._Automaton = _BuildAutomaton_21()




def _BuildAutomaton_22 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_22
    del _BuildAutomaton_22
    import pyxb.utils.fac as fac

    counters = set()
    cc_0 = fac.CounterCondition(min=2, max=2, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 961, 10))
    counters.add(cc_0)
    states = []
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(Divide._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'operand')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 961, 10))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_0._set_transitionSet(transitions)
    return fac.Automaton(states, counters, False, containing_state=None)
Divide._Automaton = _BuildAutomaton_22()




def _BuildAutomaton_23 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_23
    del _BuildAutomaton_23
    import pyxb.utils.fac as fac

    counters = set()
    cc_0 = fac.CounterCondition(min=2, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 909, 10))
    counters.add(cc_0)
    states = []
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(Min._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'operand')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 909, 10))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_0._set_transitionSet(transitions)
    return fac.Automaton(states, counters, False, containing_state=None)
Min._Automaton = _BuildAutomaton_23()




def _BuildAutomaton_24 ():
    # Remove this helper function from the namespace after it is invoked
    global _BuildAutomaton_24
    del _BuildAutomaton_24
    import pyxb.utils.fac as fac

    counters = set()
    cc_0 = fac.CounterCondition(min=2, max=None, metadata=pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 909, 10))
    counters.add(cc_0)
    states = []
    final_update = set()
    final_update.add(fac.UpdateInstruction(cc_0, False))
    symbol = pyxb.binding.content.ElementUse(Max._UseForTag(pyxb.namespace.ExpandedName(Namespace, 'operand')), pyxb.utils.utility.Location('/home/kenny/Desktop/HBP/Experiments/bibi_configuration.xsd', 909, 10))
    st_0 = fac.State(symbol, is_initial=True, final_update=final_update, is_unordered_catenation=False)
    states.append(st_0)
    transitions = []
    transitions.append(fac.Transition(st_0, [
        fac.UpdateInstruction(cc_0, True) ]))
    st_0._set_transitionSet(transitions)
    return fac.Automaton(states, counters, False, containing_state=None)
Max._Automaton = _BuildAutomaton_24()
