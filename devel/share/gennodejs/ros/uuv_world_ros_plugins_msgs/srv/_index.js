
"use strict";

let TransformToSphericalCoord = require('./TransformToSphericalCoord.js')
let GetCurrentModel = require('./GetCurrentModel.js')
let SetCurrentDirection = require('./SetCurrentDirection.js')
let TransformFromSphericalCoord = require('./TransformFromSphericalCoord.js')
let SetCurrentVelocity = require('./SetCurrentVelocity.js')
let GetOriginSphericalCoord = require('./GetOriginSphericalCoord.js')
let SetOriginSphericalCoord = require('./SetOriginSphericalCoord.js')
let SetCurrentModel = require('./SetCurrentModel.js')

module.exports = {
  TransformToSphericalCoord: TransformToSphericalCoord,
  GetCurrentModel: GetCurrentModel,
  SetCurrentDirection: SetCurrentDirection,
  TransformFromSphericalCoord: TransformFromSphericalCoord,
  SetCurrentVelocity: SetCurrentVelocity,
  GetOriginSphericalCoord: GetOriginSphericalCoord,
  SetOriginSphericalCoord: SetOriginSphericalCoord,
  SetCurrentModel: SetCurrentModel,
};
