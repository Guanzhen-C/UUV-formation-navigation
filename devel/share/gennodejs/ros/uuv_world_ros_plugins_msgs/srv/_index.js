
"use strict";

let SetCurrentVelocity = require('./SetCurrentVelocity.js')
let TransformFromSphericalCoord = require('./TransformFromSphericalCoord.js')
let GetOriginSphericalCoord = require('./GetOriginSphericalCoord.js')
let SetOriginSphericalCoord = require('./SetOriginSphericalCoord.js')
let TransformToSphericalCoord = require('./TransformToSphericalCoord.js')
let SetCurrentModel = require('./SetCurrentModel.js')
let SetCurrentDirection = require('./SetCurrentDirection.js')
let GetCurrentModel = require('./GetCurrentModel.js')

module.exports = {
  SetCurrentVelocity: SetCurrentVelocity,
  TransformFromSphericalCoord: TransformFromSphericalCoord,
  GetOriginSphericalCoord: GetOriginSphericalCoord,
  SetOriginSphericalCoord: SetOriginSphericalCoord,
  TransformToSphericalCoord: TransformToSphericalCoord,
  SetCurrentModel: SetCurrentModel,
  SetCurrentDirection: SetCurrentDirection,
  GetCurrentModel: GetCurrentModel,
};
