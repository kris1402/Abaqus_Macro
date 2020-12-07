# -*- coding: mbcs -*-
# Do not delete the following import lines
from abaqus import *
from abaqusConstants import *
import __main__


def cylinderFinal(radius, depth,DisplacementValueIn,density,youngModule,poissonRation,jcA,jcB,jcN,jcM,meltingTemp,transitionTemp,jkC,epsilonDot,name,displacement):
    import section
    import regionToolset
    import displayGroupMdbToolset as dgm
    import part
    import material
    import assembly
    import step
    import interaction
    import load
    import mesh
    import optimization
    import job
    import sketch
    import visualization
    import xyPlot
    import displayGroupOdbToolset as dgo
    import connectorBehavior
    from abaqus import getWarningReply, YES, NO

    if (radius > 0 and depth > 0):
        #Displacement value
        DisplacementValue = (DisplacementValueIn * depth) / 100
        #set name upsetting
        mdb.models.changeKey(fromName='Model-1', toName='upsetting')
        session.viewports['Viewport: 1'].setValues(displayedObject=None)
        s = mdb.models['upsetting'].ConstrainedSketch(name='__profile__',
            sheetSize=200.0)
        g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
        s.setPrimaryObject(option=STANDALONE)
        #Set the Circle radius
        s.CircleByCenterPerimeter(center=(0.0, 0.0), point1=(-17.5, 18.75))
        s.RadialDimension(curve=g[2], textPoint=(44.4774856567383, -9.54545211791992),
            radius=radius)
        p = mdb.models['upsetting'].Part(name='probe', dimensionality=THREE_D,
            type=DEFORMABLE_BODY)
        p = mdb.models['upsetting'].parts['probe']
        #Solid Extrution
        p.BaseSolidExtrude(sketch=s, depth=depth)
        s.unsetPrimaryObject()
        p = mdb.models['upsetting'].parts['probe']
        session.viewports['Viewport: 1'].setValues(displayedObject=p)
        del mdb.models['upsetting'].sketches['__profile__']

        s1 = mdb.models['upsetting'].ConstrainedSketch(name='__profile__',
            sheetSize=200.0)
        g, v, d, c = s1.geometry, s1.vertices, s1.dimensions, s1.constraints
        s1.setPrimaryObject(option=STANDALONE)
        s1.ConstructionLine(point1=(0.0, -100.0), point2=(0.0, 100.0))
        s1.FixedConstraint(entity=g[2])
        s1.Line(point1=(0.0, 0.0), point2=(100.0, 0.0))
        s1.HorizontalConstraint(entity=g[3], addUndoState=False)
        s1.PerpendicularConstraint(entity1=g[2], entity2=g[3], addUndoState=False)
        s1.CoincidentConstraint(entity1=v[0], entity2=g[2], addUndoState=False)
        #Tool Dimension
        s1.ObliqueDimension(vertex1=v[0], vertex2=v[1], textPoint=(41.1336822509766,
            -18.9950370788574), value=100.0)
        p = mdb.models['upsetting'].Part(name='tool', dimensionality=THREE_D,
            type=DISCRETE_RIGID_SURFACE)
        p = mdb.models['upsetting'].parts['tool']
        #Create Tool Shell
        p.BaseShellRevolve(sketch=s1, angle=360.0, flipRevolveDirection=OFF)
        s1.unsetPrimaryObject()
        p = mdb.models['upsetting'].parts['tool']
        session.viewports['Viewport: 1'].setValues(displayedObject=p)
        del mdb.models['upsetting'].sketches['__profile__']

        session.viewports['Viewport: 1'].partDisplay.setValues(sectionAssignments=ON,
            engineeringFeatures=ON)
        session.viewports['Viewport: 1'].partDisplay.geometryOptions.setValues(
            referenceRepresentation=OFF)
        #Material Steel parameters
        mdb.models['upsetting'].Material(name='Steel')
        mdb.models['upsetting'].materials['Steel'].Density(table=((density, ), ))
        mdb.models['upsetting'].materials['Steel'].Elastic(table=((youngModule, poissonRation), ))
        mdb.models['upsetting'].materials['Steel'].Plastic(hardening=JOHNSON_COOK,
            table=((jcA, jcB, jcN, jcM, meltingTemp, transitionTemp), ))
        mdb.models['upsetting'].materials['Steel'].plastic.RateDependent(
            type=JOHNSON_COOK, table=((jkC, epsilonDot), ))
        mdb.models['upsetting'].HomogeneousSolidSection(name='Section-1',
            material='Steel', thickness=None)
        a = mdb.models['upsetting'].rootAssembly
        session.viewports['Viewport: 1'].setValues(displayedObject=a)
        session.viewports['Viewport: 1'].assemblyDisplay.setValues(
            optimizationTasks=OFF, geometricRestrictions=OFF, stopConditions=OFF)
        a = mdb.models['upsetting'].rootAssembly
        a.DatumCsysByDefault(CARTESIAN)
        p = mdb.models['upsetting'].parts['probe']
        a.Instance(name='probe-1', part=p, dependent=ON)
        p = mdb.models['upsetting'].parts['tool']
        a.Instance(name='tool-1', part=p, dependent=ON)
        a = mdb.models['upsetting'].rootAssembly
        a.rotate(instanceList=('tool-1', ), axisPoint=(100.0, 0.0, 0.0),
            axisDirection=(-100.0, 0.0, 0.0), angle=90.0)
        a = mdb.models['upsetting'].rootAssembly
        p = mdb.models['upsetting'].parts['tool']
        a.Instance(name='tool-2', part=p, dependent=ON)
        a = mdb.models['upsetting'].rootAssembly
        a.translate(instanceList=('tool-2', ), vector=(0.0, 0.0, 150.0))
        a = mdb.models['upsetting'].rootAssembly
        a.rotate(instanceList=('tool-2', ), axisPoint=(0.0, 0.0, 150.0),
            axisDirection=(100.0, 0.0, 0.0), angle=90.0)
        p1 = mdb.models['upsetting'].parts['probe']
        session.viewports['Viewport: 1'].setValues(displayedObject=p1)
        p = mdb.models['upsetting'].parts['probe']
        c = p.cells
        cells = c.getSequenceFromMask(mask=('[#1 ]', ), )
        region = p.Set(cells=cells, name='Set-1')
        p = mdb.models['upsetting'].parts['probe']
        p.SectionAssignment(region=region, sectionName='Section-1', offset=0.0,
            offsetType=MIDDLE_SURFACE, offsetField='',
            thicknessAssignment=FROM_SECTION)
        p = mdb.models['upsetting'].parts['probe']
        c = p.cells
        cells = c.getSequenceFromMask(mask=('[#1 ]', ), )
        p.Set(cells=cells, name='all')

        p = mdb.models['upsetting'].parts['tool']
        v2, e1, d2, n1 = p.vertices, p.edges, p.datums, p.nodes
        p.ReferencePoint(point=v2[0])
        p1 = mdb.models['upsetting'].parts['probe']
        session.viewports['Viewport: 1'].setValues(displayedObject=p1)

        p = mdb.models['upsetting'].parts['probe']
        s = p.faces
        side1Faces = s.getSequenceFromMask(mask=('[#1 ]',), )
        p.Surface(side1Faces=side1Faces, name='Surf-1')
        del mdb.models['upsetting'].parts['probe'].surfaces['Surf-1']
        session.viewports['Viewport: 1'].partDisplay.setValues(sectionAssignments=ON,
                                                               engineeringFeatures=ON)
        session.viewports['Viewport: 1'].partDisplay.geometryOptions.setValues(
            referenceRepresentation=OFF)
        p1 = mdb.models['upsetting'].parts['probe']
        session.viewports['Viewport: 1'].setValues(displayedObject=p1)
        a = mdb.models['upsetting'].rootAssembly
        a.regenerate()
        session.viewports['Viewport: 1'].setValues(displayedObject=a)
        session.viewports['Viewport: 1'].assemblyDisplay.setValues(interactions=OFF,
                                                                   constraints=OFF, connectors=OFF, engineeringFeatures=OFF)
        p1 = mdb.models['upsetting'].parts['probe']
        session.viewports['Viewport: 1'].setValues(displayedObject=p1)
        a = mdb.models['upsetting'].rootAssembly
        session.viewports['Viewport: 1'].setValues(displayedObject=a)
        session.viewports['Viewport: 1'].assemblyDisplay.setValues(
            adaptiveMeshConstraints=ON)
        #Set the method of calculation
        mdb.models['upsetting'].ExplicitDynamicsStep(name='Step-1', previous='Initial',
                                                     massScaling=((SEMI_AUTOMATIC, MODEL, AT_BEGINNING, 1000.0, 0.0, None,
                                                                   0, 0, 0.0, 0.0, 0, None),), improvedDtMethod=ON)
        session.viewports['Viewport: 1'].assemblyDisplay.setValues(step='Step-1')
        #Set the results
        mdb.models['upsetting'].fieldOutputRequests['F-Output-1'].setValues(variables=(
            'S', 'SVAVG', 'PE', 'PEEQ', 'PEEQVAVG', 'LE', 'U', 'V', 'RF',
            'CSTRESS'))
        session.viewports['Viewport: 1'].assemblyDisplay.setValues(interactions=ON,
                                                                   constraints=ON, connectors=ON, engineeringFeatures=ON,
                                                                   adaptiveMeshConstraints=OFF)

        session.viewports['Viewport: 1'].partDisplay.setValues(sectionAssignments=OFF,
                                                               engineeringFeatures=OFF)
        session.viewports['Viewport: 1'].partDisplay.geometryOptions.setValues(
            referenceRepresentation=ON)
        p1 = mdb.models['upsetting'].parts['probe']
        session.viewports['Viewport: 1'].setValues(displayedObject=p1)
        a = mdb.models['upsetting'].rootAssembly
        session.viewports['Viewport: 1'].setValues(displayedObject=a)
        session.viewports['Viewport: 1'].assemblyDisplay.setValues(interactions=OFF,
                                                                   constraints=OFF, connectors=OFF, engineeringFeatures=OFF)
        p1 = mdb.models['upsetting'].parts['probe']
        session.viewports['Viewport: 1'].setValues(displayedObject=p1)
        p = mdb.models['upsetting'].parts['probe']
        s = p.faces
        side1Faces = s.getSequenceFromMask(mask=('[#2 ]',), )
        p.Surface(side1Faces=side1Faces, name='upper')
        p = mdb.models['upsetting'].parts['probe']
        s = p.faces
        side1Faces = s.getSequenceFromMask(mask=('[#4 ]',), )
        p.Surface(side1Faces=side1Faces, name='lower')

        a1 = mdb.models['upsetting'].rootAssembly
        a1.regenerate()
        a = mdb.models['upsetting'].rootAssembly
        session.viewports['Viewport: 1'].setValues(displayedObject=a)
        session.viewports['Viewport: 1'].assemblyDisplay.setValues(interactions=ON,
                                                                   constraints=ON, connectors=ON, engineeringFeatures=ON)
        a = mdb.models['upsetting'].rootAssembly
        session.viewports['Viewport: 1'].setValues(displayedObject=a)
        mdb.models['upsetting'].ContactProperty('IntProp-1')
        mdb.models['upsetting'].interactionProperties['IntProp-1'].TangentialBehavior(
            formulation=PENALTY, directionality=ISOTROPIC, slipRateDependency=OFF,
            pressureDependency=OFF, temperatureDependency=OFF, dependencies=0,
            table=((0.3,),), shearStressLimit=None, maximumElasticSlip=FRACTION,
            fraction=0.005, elasticSlipStiffness=None)
        a = mdb.models['upsetting'].rootAssembly
        s1 = a.instances['tool-2'].faces
        side1Faces1 = s1.getSequenceFromMask(mask=('[#1 ]',), )
        region1 = a.Surface(side1Faces=side1Faces1, name='m_Surf-1')
        a = mdb.models['upsetting'].rootAssembly
        region2 = a.instances['probe-1'].surfaces['upper']
        #Contact tool and model
        mdb.models['upsetting'].SurfaceToSurfaceContactExp(name='Int-1',
                                                           createStepName='Step-1', master=region1, slave=region2,
                                                           mechanicalConstraint=KINEMATIC, sliding=FINITE,
                                                           interactionProperty='IntProp-1', initialClearance=OMIT,
                                                           datumAxis=None,
                                                           clearanceRegion=None)

        a = mdb.models['upsetting'].rootAssembly
        s1 = a.instances['tool-1'].faces
        side1Faces1 = s1.getSequenceFromMask(mask=('[#1 ]',), )
        region1 = a.Surface(side1Faces=side1Faces1, name='m_Surf-2')
        a = mdb.models['upsetting'].rootAssembly
        region2 = a.instances['probe-1'].surfaces['lower']
        mdb.models['upsetting'].SurfaceToSurfaceContactExp(name='Int-2',
                                                           createStepName='Step-1', master=region1, slave=region2,
                                                           mechanicalConstraint=KINEMATIC, sliding=FINITE,
                                                           interactionProperty='IntProp-1', initialClearance=OMIT,
                                                           datumAxis=None,
                                                           clearanceRegion=None)
        session.viewports['Viewport: 1'].assemblyDisplay.setValues(loads=ON, bcs=ON,
                                                                   predefinedFields=ON, interactions=OFF, constraints=OFF,
                                                                   engineeringFeatures=OFF)

        a = mdb.models['upsetting'].rootAssembly
        r1 = a.instances['tool-1'].referencePoints
        refPoints1 = (r1[2],)
        #Set reference points
        region = a.Set(referencePoints=refPoints1, name='Set-1')
        mdb.models['upsetting'].DisplacementBC(name='BC-1', createStepName='Step-1',
                                               region=region, u1=0.0, u2=0.0, u3=0.0, ur1=0.0, ur2=0.0, ur3=0.0,
                                               amplitude=UNSET, fixed=OFF, distributionType=UNIFORM, fieldName='',
                                               localCsys=None)
        mdb.models['upsetting'].TabularAmplitude(name='Amp-1', timeSpan=STEP,
                                                 smooth=SOLVER_DEFAULT, data=((0.0, 0.0), (1.0, 1.0)))
        a = mdb.models['upsetting'].rootAssembly
        r1 = a.instances['tool-2'].referencePoints
        refPoints1 = (r1[2],)
        #Set reference point and displacement
        region = a.Set(referencePoints=refPoints1, name='Set-2')
        mdb.models['upsetting'].DisplacementBC(name='BC-2', createStepName='Step-1',
                                               region=region, u1=0.0, u2=0.0, u3=-DisplacementValue, ur1=0.0, ur2=0.0, ur3=0.0,
                                               amplitude='Amp-1', fixed=OFF, distributionType=UNIFORM, fieldName='',
                                               localCsys=None)
        session.viewports['Viewport: 1'].partDisplay.setValues(mesh=ON)
        session.viewports['Viewport: 1'].partDisplay.meshOptions.setValues(
            meshTechnique=ON)
        session.viewports['Viewport: 1'].partDisplay.geometryOptions.setValues(
            referenceRepresentation=OFF)
        p1 = mdb.models['upsetting'].parts['probe']
        session.viewports['Viewport: 1'].setValues(displayedObject=p1)
        p = mdb.models['upsetting'].parts['probe']
        p.seedPart(size=14.0, deviationFactor=0.1, minSizeFactor=0.1)
        p = mdb.models['upsetting'].parts['probe']
        p.generateMesh()
        elemType1 = mesh.ElemType(elemCode=C3D8R, elemLibrary=EXPLICIT,
                                  kinematicSplit=AVERAGE_STRAIN, secondOrderAccuracy=OFF,
                                  hourglassControl=DEFAULT, distortionControl=DEFAULT)
        elemType2 = mesh.ElemType(elemCode=C3D6, elemLibrary=EXPLICIT)
        elemType3 = mesh.ElemType(elemCode=C3D4, elemLibrary=EXPLICIT)
        p = mdb.models['upsetting'].parts['probe']
        c = p.cells
        cells = c.getSequenceFromMask(mask=('[#1 ]',), )
        pickedRegions = (cells,)
        p.setElementType(regions=pickedRegions, elemTypes=(elemType1, elemType2,
                                                           elemType3))
        p1 = mdb.models['upsetting'].parts['tool']
        session.viewports['Viewport: 1'].setValues(displayedObject=p1)
        p = mdb.models['upsetting'].parts['tool']
        p.seedPart(size=28.0, deviationFactor=0.1, minSizeFactor=0.1)
        p = mdb.models['upsetting'].parts['tool']
        p.generateMesh()
        a = mdb.models['upsetting'].rootAssembly
        a.regenerate()
        session.viewports['Viewport: 1'].setValues(displayedObject=a)
        session.viewports['Viewport: 1'].assemblyDisplay.setValues(loads=OFF, bcs=OFF,
                                                                   predefinedFields=OFF, connectors=OFF)
        #Create Job
        mdb.Job(name='HF', model='upsetting', description='', type=ANALYSIS,
                atTime=None, waitMinutes=0, waitHours=0, queue=None, memory=90,
                memoryUnits=PERCENTAGE, explicitPrecision=SINGLE,
                nodalOutputPrecision=SINGLE, echoPrint=OFF, modelPrint=OFF,
                contactPrint=OFF, historyPrint=OFF, userSubroutine='', scratch='',
                resultsFormat=ODB)
        mdb.jobs['HF'].submit(consistencyChecking=OFF)
        o3 = session.openOdb(name='HF.odb')
        session.viewports['Viewport: 1'].setValues(displayedObject=o3)
        session.viewports['Viewport: 1'].makeCurrent()
        session.mdbData.summary()
        session.viewports['Viewport: 1'].odbDisplay.display.setValues(plotState=(
            CONTOURS_ON_DEF,))

        session.viewports['Viewport: 1'].setValues(displayedObject=session.odbs['HF.odb'])
        session.viewports['Viewport: 1'].odbDisplay.setPrimaryVariable(
            variableLabel='U', outputPosition=NODAL, refinement=(INVARIANT,
        'Magnitude'), )

        #--------Save--------
        if displacement == True:
            a = mdb.models['upsetting'].rootAssembly
            session.viewports['Viewport: 1'].setValues(displayedObject=a)
            o3 = session.openOdb(name='HF.odb')
            session.viewports['Viewport: 1'].setValues(displayedObject=o3)
            session.viewports['Viewport: 1'].makeCurrent()
            a = mdb.models['upsetting'].rootAssembly
            session.viewports['Viewport: 1'].setValues(displayedObject=a)
            session.mdbData.summary()
            session.viewports['Viewport: 1'].setValues(
                displayedObject=session.odbs['HF.odb'])
            session.linkedViewportCommands.setValues(_highlightLinkedViewports=False)
            odb = session.odbs['HF.odb']
            session.xyDataListFromField(odb=odb, outputPosition=NODAL, variable=(('U',
                                                                                  NODAL, ((COMPONENT, 'U3'),)),),
                                        nodeSets=("SET-2",))
            session.xyDataObjects.changeKey(fromName='U:U3 PI: TOOL-2 N: 69', toName=name)
            x0 = session.xyDataObjects[name]
            session.writeXYReport(fileName= str(name)+'.txt', xyData=(x0,))



    else:
        getWarningReply(message='Bledy wymiar, wprowadz poprawna wartosc!', buttons=(YES, NO))



