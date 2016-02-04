// GriffpatchBox2D.js
// Griffpatch, January 2016
// Extension to enable Box2D integration in Scratch

(function (ext) {

	var box2dscript = box2dscript;
	if (!box2dscript) {
		box2dscript = document.createElement("script");
		box2dscript.type="text/javascript";
		box2dscript.src="http://griffpatch.github.io/Box2D.js-Scratch2-Extension/Box2d.min.js";
		//box2dscript.src=document.extURLs.box2d;
		document.body.appendChild(box2dscript);
	}

	ext.available = function() {
		return !!Box2D;
	}
	
    ext._stop = function () {};
    ext._shutdown = function () {};

	var b2Vec2, b2AABB, b2BodyDef, b2Body, b2FixtureDef, b2Fixture, b2World, b2MassData, b2PolygonShape, b2CircleShape, b2DebugDraw, b2MouseJointDef;
	var world, fixDef, zoom;

	var fixDef;
	var bodyDef;

	var uid_seq = 0;
	var ujid_seq = 0;

	var bodies = {};
	var joints = {};

	var categorySeq = 1;
	var categories = {'default':1}
	
	var bodyCategoryBits = 1;
	var bodyMaskBits = 1;
	var noCollideSeq = 0;
	
	const toRad = Math.PI / 180;

    ext._getStatus = function () {
		if (Box2D) {
			return {status: 2, msg: ' Ready'};
		} else {
			return {status: 1, msg: ' Failed to load Box2D'};
		}
    };
	
	ext.init = function( scale, gravity, scene ) {
		b2Vec2 = Box2D.Common.Math.b2Vec2;
		b2AABB = Box2D.Collision.b2AABB;
		b2BodyDef = Box2D.Dynamics.b2BodyDef;
		b2Body = Box2D.Dynamics.b2Body;
		b2FixtureDef = Box2D.Dynamics.b2FixtureDef;
		b2Fixture = Box2D.Dynamics.b2Fixture;
		b2World = Box2D.Dynamics.b2World;
		b2MassData = Box2D.Collision.Shapes.b2MassData;
		b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape;
		b2CircleShape = Box2D.Collision.Shapes.b2CircleShape;
		b2DebugDraw = Box2D.Dynamics.b2DebugDraw;
		b2MouseJointDef =  Box2D.Dynamics.Joints.b2MouseJointDef;

		world = new b2World(
			new b2Vec2(0, gravity)    //gravity (10)
		 ,  true                 //allow sleep
		);
	
		zoom = scale;
	
		fixDef = new b2FixtureDef;
		fixDef.density = 1.0;		// 1.0
		fixDef.friction = 0.5;		// 0.5
		fixDef.restitution = 0.2;	// 0.2

		bodyDef = new b2BodyDef;

		if (scene=='stage') {

			//create ground
			bodyDef.type = b2Body.b2_staticBody;
			fixDef.shape = new b2PolygonShape;
			fixDef.shape.SetAsBox(250/zoom, 10/zoom);
			bodyDef.position.Set(0,-190/zoom);
			world.CreateBody(bodyDef).CreateFixture(fixDef);
			bodyDef.position.Set(0,1000/zoom);
			world.CreateBody(bodyDef).CreateFixture(fixDef);
			fixDef.shape.SetAsBox(10/zoom, 800/zoom);
			bodyDef.position.Set(-250/zoom,540/zoom);
			world.CreateBody(bodyDef).CreateFixture(fixDef);
			bodyDef.position.Set(250/zoom,540/zoom);
			world.CreateBody(bodyDef).CreateFixture(fixDef);
		}
		
		bodies = {};
		joints = {};
		uid_seq = 0;
		ujid_seq = 0;
		
		categorySeq = 1;
		categories = {'default':1}
		bodyCategoryBits = 1;
		noCollideSeq = 0;

		bodyDef.type = b2Body.b2_dynamicBody;
	};

	ext.setBodyAttrs = function(stat, dens, fric, rest) {
		bodyDef.type = stat==='static' ? b2Body.b2_staticBody : b2Body.b2_dynamicBody;
		fixDef.density = dens;		// 1.0
		fixDef.friction = fric;		// 0.5
		fixDef.restitution = rest;	// 0.2
	};
	
	ext.setBodyAttr = function(attr, bodyIDs, val) {
		var bds = bodyIDs.split(' ');
		for (var i=0; i<bds.length; i++) {
			var id = bds[i];
			if (id.length>0) {
				var body = bodies[id];
				if (body) {
					switch (attr) {
						case 'damping': body.SetLinearDamping( val ); break;
						case 'rotational damping': body.GetAngularDamping( val ); break;
					}
				}
			}
		}
	};
	
	ext.defineCategory = function(categoryIDs) {
		var cids = categoryIDs.split(' ');
		bodyCategoryBits = 0;
		for (var i=0; i<cids.length; i++) {
			var cid = cids[i];
			if (cid.length>0) {
				var cat = categories[cid];
				if (!cat) {
					cat = categories[cid] = categorySeq = categorySeq*2;
				}
				bodyCategoryBits |= cat;
			}
		}
	};
	
	ext.defineMask = function(categoryIDs) {
		var cids = categoryIDs.split(' ');
		bodyMaskBits = 0;
		for (var i=0; i<cids.length; i++) {
			var cid = cids[i];
			if (cid.length>0) {
				var cat = categories[cid];
				if (!cat) {
					cat = categories[cid] = categorySeq = categorySeq*2;
				}
				bodyMaskBits |= cat;
			}
		}
	};
	
	ext.definePoly = function(points) {
		fixDef.shape = new b2PolygonShape;
		
		var pts = points.split(' ');
		for (var i = 0; i < pts.length; i++) {
			if (pts[i].length==0) {         
				pts.splice(i, 1);
				i--;
			}
		}

		// console.log(pts);
		
		var vertices = [];
		
		for (var i=pts.length; i>0;i-=2) {
			vertices.push( new b2Vec2(parseFloat(pts[i-2])/zoom, parseFloat(pts[i-1])/zoom) );
		}
		
		// console.log(vertices);
		
		fixDef.shape.SetAsArray( vertices );
	};

	ext.defineRect = function(w,h) {
		fixDef.shape = new b2PolygonShape;
		fixDef.shape.SetAsBox( w/2/zoom, h/2/zoom );
	};

	ext.defineCircle = function(d) {
		fixDef.shape = new b2CircleShape;
		fixDef.shape.SetRadius( d/2/zoom );
	};

	ext.placeBody = function(id, x,y, dir) {
		if (bodies[id]) {
			world.DestroyBody( bodies[id] );
		}
		
		fixDef.filter.categoryBits = bodyCategoryBits;
		fixDef.filter.maskBits = bodyMaskBits;
		
		bodyDef.position.x = x/zoom;
		bodyDef.position.y = y/zoom;
		bodyDef.angle = (90-dir)*toRad;
		var body = world.CreateBody(bodyDef);
		body.uid = id;
		body.CreateFixture(fixDef);
		bodies[id] = body;
	};
	
	ext.destroyBody = function(id) {
		if (bodies[id]) {
			world.DestroyBody( bodies[id] );
			delete bodies[id];
		}
	};

	ext.getBodyAttr = function(attr, id) {
		var body = bodies[id];
		if (!body) return '';
		switch (attr) {
			case 'x': return body.GetPosition().x * zoom;
			case 'y': return body.GetPosition().y * zoom;
			case 'direction': return 90-(body.GetAngle()/toRad);
			case 'awake': return body.IsAwake() ? 1 : 0;
		}
		return '';
	};

	var mousePVec, selectedBody;
	
	function getBodyCB(fixture) {
		if(fixture.GetBody().GetType() != b2Body.b2_staticBody) {
			if(fixture.GetShape().TestPoint(fixture.GetBody().GetTransform(), mousePVec)) {
				selectedBody = fixture.GetBody();
				return false;
			}
		}
		return true;
	};

	ext.getBodyIDAt = function(x,y) {
		mousePVec = new b2Vec2(x/zoom, y/zoom);
		var aabb = new b2AABB();
		aabb.lowerBound.Set(mousePVec.x - 0.001, mousePVec.y - 0.001);
		aabb.upperBound.Set(mousePVec.x + 0.001, mousePVec.y + 0.001);

		// Query the world for overlapping shapes.
		selectedBody = null;
		world.QueryAABB(getBodyCB, aabb);
		
		return selectedBody ? selectedBody.uid : '';
	};

/*	ext.createJointBetween = function(bodyID, x,y, bodyID2, x2,y2) {
		if (bodyID=='') {
			bodyID = null;
		}
		var body = bodyID ? bodies[bodyID] : world.GetGroundBody();
		var body2 = bodies[bodyID2];
		
		if (body) {
			var md = new Box2D.Dynamics.Joints.b2RevoluteJointDef();
			md.bodyA = body;
			md.bodyB = body2;
			md.localAnchorA = {x:x/zoom, y:y/zoom};
			md.localAnchorB = {x:x2/zoom, y:y2/zoom};
			//md.collideConnected = true;
			//md.maxForce = 300.0 * body.GetMass();
			var joint = world.CreateJoint(md);
			if (bodyID) {
				body.SetAwake(true);
			}
			body2.SetAwake(true);
			joints[++ujid_seq] = joint;
			return ''+ujid_seq;
		}
		return '';
	};*/
	
	// ['',  'Define Spring Length: %n  Damping: %n  Freq: %n',				'defineSpring',		100, 0.5, 8],
	var defSpring = {len:100, damp:0.7, freq: 5};
	ext.defineSpring = function( len, damp, freq ) {
		defSpring.len = len<0.1 ? 0.1 : len / zoom;
		defSpring.damp = damp<0 ? 0.7 : damp;
		defSpring.freq = freq>0 ? freq : 5;
	}
	
	ext.createJointOfType = function(jName, typ, bodyID, x,y, bodyID2, x2,y2) {
		
		if (jName.length>0) ext.destroyJoint(jName);

		if (bodyID=='') bodyID = null;
		if (bodyID2=='') bodyID2 = null;
		if (!bodyID && !bodyID2) return '';
			
		var body = bodyID ? bodies[bodyID] : world.GetGroundBody();
		var body2 = bodyID2 ? bodies[bodyID2] : world.GetGroundBody();
		
		if (!body || !body2) return '';
		
		var md;
		switch (typ) {
			case 'Spring':
				md = new Box2D.Dynamics.Joints.b2DistanceJointDef();
				md.length = defSpring.len;
				md.dampingRatio = defSpring.damp;
				md.frequencyHz = defSpring.freq;
				md.bodyA = body;
				md.bodyB = body2;
				md.localAnchorA = {x:x/zoom, y:y/zoom};
				md.localAnchorB = {x:x2/zoom, y:y2/zoom};
				break;
				
			case 'Rotating':
				md = new Box2D.Dynamics.Joints.b2RevoluteJointDef();
				md.bodyA = body;
				md.bodyB = body2;
				md.localAnchorA = {x:x/zoom, y:y/zoom};
				md.localAnchorB = {x:x2/zoom, y:y2/zoom};
				break;
				
			case 'Mouse':
				var md = new b2MouseJointDef();
				if (bodyID=='') {
					md.bodyB = body2;
					md.target.Set(x2/zoom, y2/zoom);
				} else {
					md.bodyB = body;
					md.target.Set(x/zoom, y/zoom);
				}
				md.bodyA = world.GetGroundBody();
				md.collideConnected = true;
				md.maxForce = 300.0 * body.GetMass();
				break;
		}
		
		//md.collideConnected = true;
		//md.maxForce = 300.0 * body.GetMass();
		var joint = world.CreateJoint(md);
		if (bodyID.length>0) {
			body.SetAwake(true);
		}
		if (bodyID2.length>0) {
			body2.SetAwake(true);
		}
		
		if (jName.length==0) jName = '_'+(++ujid_seq);
		joints[jName] = joint;
	};
	
	/*ext.createJointAt = function(bodyID, x, y) {
		var body = bodies[bodyID];
		if (body) {
			var md = new b2MouseJointDef();
			md.bodyA = world.GetGroundBody();
			md.bodyB = body;
			md.target.Set(x/zoom, y/zoom);
			md.collideConnected = true;
			md.maxForce = 300.0 * body.GetMass();
			var joint = world.CreateJoint(md);
			body.SetAwake(true);
			joints[++ujid_seq] = joint;
			return ''+ujid_seq;
		}
		return '';
	};*/
	
	ext.setJointTarget = function(jointID, x, y) {
		var joint = joints[jointID];
		if (joint) {
			joint.SetTarget(new b2Vec2(x/zoom, y/zoom));
		}
	};
		
	ext.setJointAttr = function(attr, jointID, val) {
		// JointAttr: ['Motor On','Motor Speed','Max Torque', 'Limits On','Lower Limit','Upper Limit'],

		var jointids = jointID.split(' ');
		for (var i=0;i<jointids.length;i++) {
			var joint = joints[jointids[i]];
			if (joint) {
				switch(attr) {
				case 'Motor On': joint.EnableMotor(val>0); break;
				case 'Motor Speed': joint.SetMotorSpeed(val); break;
				case 'Max Torque': joint.SetMaxMotorTorque(val); break;
				
				case 'Limits On': joint.EnableLimit(val>0); break;
				case 'Lower Limit': joint.SetLimits(joint.GetJointAngle()+val*toRad, joint.GetUpperLimit()); break;
				case 'Upper Limit': joint.SetLimits(joint.GetLowerLimit(),joint.GetJointAngle()+val*toRad); break;
			}
			}
		}
	};
		
	ext.getJointAttr = function(attr, jointID) {
		// JointAttrRead: ['Angle','Speed','Motor Torque', 'Reaction Torque'],

		var joint = joints[jointID];
		if (joint) {
			switch(attr) {
				case 'Angle': return joint.GetJointAngle()/toRad;
				case 'Speed': return joint.GetJointSpeed();
				case 'Motor Torque': return joint.GetMotorTorque();
				case 'Reaction Torque': return joint.GetReactionTorque();
				
//				case 'Lower Limit': return joint.GetLowerLimit()/toRad;
//				case 'Upper Limit': return joint.GetUpperLimit()/toRad;
			}
		}
	};
		
	ext.destroyJoint = function(jointID) {
		var joint = joints[jointID];
		if (joint) {
			world.DestroyJoint(joint);
			delete joints[jointID];
		}
	};

	ext.applyForceToBody = function(ftype, bodyID, x, y, pow, dir) {
		var body = bodies[bodyID];
		if (!body)
			return;

		dir = (90-dir)*toRad;
			
		if (ftype==='Impulse') {
			body.ApplyImpulse( {x:pow*Math.cos(dir),y:pow*Math.sin(dir)}, body.GetWorldPoint({x:x/zoom,y:y/zoom}) );			
		} else if (ftype==='World Impulse') {
			body.ApplyForce( {x:pow*Math.cos(dir),y:pow*Math.sin(dir)}, {x:x/zoom,y:y/zoom} );			
		}
	};
	
	ext.applyAngForceToBody = function(ftype, bodyID, pow) {
		var body = bodies[bodyID];
		if (!body)
			return;

		if (ftype==='Impulse') {
			//console.log(body);
			body.ApplyTorque( -pow );			
		}
	};
	
	ext.createNoCollideSet = function(set) {
		noCollideSeq--;
		var bids = set.split(' ');
		for (var i=0; i<bids.length; i++) {
			var bid = bids[i];
			if (bid.length>0) {
				var body = bodies[bid];
				if (body) {
					var fix = body.GetFixtureList();
					while (fix) {
						var fdata = fix.GetFilterData();
						fdata.groupIndex = noCollideSeq;
						fix.SetFilterData(fdata);
						fix = fix.GetNext();
					}
				}
			}
		}
	};
	
	ext.stepSimulation = function() {
		world.Step(1/30, 10, 10);
		world.ClearForces();
	};
	
    var descriptor = {
        blocks: [
            ['b', 'Box2D Available',												'available'],
            ['',  'Init World; scale 1m= %n  gravity= %n  scene= %m.sceneType',		'init',			50, -10, 'stage'],
			["-"],
			["-"],
			['',  'Define Type %m.BodyTypePK  Density %n  Friction %n  Bounce %n',	'setBodyAttrs',	'dynamic', 1.0, 0.5, 0.2],
			["-"],
/*			['',  'Define Category %s',												'defineCategory',	'default'],
			['',  'Define Collide Category %s',										'defineCollideCategory',	'default'],
			["-"],*/
			['',  'Define Circle, size: %n',										'defineCircle',	100],
			['',  'Define Box, width: %n height: %n',								'defineRect',	100, 100],
			['',  'Define Polygon, points: %s',										'definePoly',	'0 50   40 -50   -40 -50'],
			["-"],
			['',  'Create Body %s  at x: %n  y: %n  dir: %n',						'placeBody',	'name', 0,0, 90],
			['',  'Create no collide set %s',										'createNoCollideSet',	'name1 name2 name3'],
			['',  'Destroy Body %s',												'destroyBody',	'name'],
			["-"],
			["-"],
			["-"],
			['',  'Set %m.bodyAttr  of body %s to %n',								'setBodyAttr',	'damping', 'name', '0.1'],
			['r', 'Get %m.bodyAttrRead  from body %s',								'getBodyAttr',	'x',	'name'],
			["-"],
			['r', 'Get id of body at x: %n  y: %n',									'getBodyIDAt',	0, 0],
			["-"],
			["-"],
			["-"],
			['',  'Apply %m.ForceType to Body %s  at x: %n  y: %n  power: %n  dir: %n',	'applyForceToBody',	'Impulse', 'name', 0, 0, 50, 90],
			['',  'Apply Angular %m.ForceType to Body %s  power: %n',				'applyAngForceToBody',	'Impulse', 'name', 0],
			["-"],
			["-"],
			["-"],
			['',  'Define Spring Length: %n  Damping: %n  Freq: %n',				'defineSpring',		100, 0.7, 5],
			['',  'Create Joint %s of type %m.JointType between %s at %n %n and %s at %n %n',	'createJointOfType', 'JointID', 'Rotating', 'BodyID', 0,0, 'BodyID', 0,0],
			['',  'Destroy Joint ID %s',											'destroyJoint',		'Joint ID'],
			["-"],
			['',  'Set Joint %m.JointAttr of joint %s to %n',						'setJointAttr',		'Motor On', 'Joint ID', 0, 0],
			['r', 'Get Joint %m.JointAttrRead of joint %s',							'getJointAttr',		'Angle', 'Joint ID'],
			['',  'Set Mouse Joint Target %s to x: %n  y: %n',						'setJointTarget',	'Joint ID', 0, 0],
			["-"],
			["-"],
			["-"],
			['',  'Step Simulation',												'stepSimulation'],
        ],
		menus: {
			sceneType: ['stage', 'nothing'],
			BodyTypePK: ['dynamic', 'static'],
			bodyAttr: ['damping', 'rotational damping'],
			bodyAttrRead: ['x', 'y', 'direction', 'awake'],
			ForceType: ['Impulse', 'World Impulse'],
			AngForceType: ['Impulse'],
			JointType: ['Rotating','Spring','Mouse'],
			JointAttr: ['Motor On','Motor Speed','Max Torque', 'Limits On','Lower Limit','Upper Limit'],
			JointAttrRead: ['Angle','Speed','Motor Torque', 'Reaction Torque'],
		},
/*        url: 'www.griffpatch.co.uk'*/
    };
	
    if (ScratchExtensions) {
//        ScratchExtensions.unregister('Griffpatch Box2D');
        ScratchExtensions.register('Griffpatch Box2D', descriptor, ext);
    }
})({});
