/**
 * Test script to compare animation behavior between:
 * 1. Creation workflow (solve, animate, cut, solve, accept, reset, animate)
 * 2. Import workflow (import JSON, accept all, reset, animate)
 *
 * Run this in the browser console after loading the app.
 */

// Helper to wait
const wait = (ms) => new Promise(resolve => setTimeout(resolve, ms));

// Helper to log state
function logAnimationState(label) {
  console.log(`\n=== ${label} ===`);
  console.log(`Mode: ${missionState.mode}`);
  console.log(`MissionReplay segments: ${missionReplay.getSegmentCount()}`);
  console.log(`MissionReplay current: ${missionReplay._currentSegmentIndex}`);
  console.log(`segmentedImport active: ${segmentedImport.isActive()}`);

  console.log(`\nstate.routes:`);
  Object.entries(state.routes || {}).forEach(([did, rd]) => {
    const traj = rd.trajectory || [];
    if (traj.length > 0) {
      console.log(`  D${did}: ${traj.length} pts, start=[${traj[0][0].toFixed(1)},${traj[0][1].toFixed(1)}], end=[${traj[traj.length-1][0].toFixed(1)},${traj[traj.length-1][1].toFixed(1)}]`);
    }
  });

  console.log(`\nstate.animation.drones:`);
  Object.entries(state.animation.drones || {}).forEach(([did, ds]) => {
    console.log(`  D${did}: animating=${ds.animating}, distTraveled=${ds.distanceTraveled?.toFixed(1)}, totalDist=${ds.totalDistance?.toFixed(1)}, segIdx=${ds.segmentIdx}`);
  });

  console.log(`\nmissionDistance: ${state.animation.missionDistance?.toFixed(1)}`);

  // Check cut distances
  console.log(`\nSegment cutDistances:`);
  for (let i = 0; i < missionReplay.getSegmentCount(); i++) {
    const seg = missionReplay.getSegment(i);
    console.log(`  Seg ${i}: cutDist=${seg?.cutDistance?.toFixed(1) || 'null'}`);
  }
}

// Test: What happens at Reset
function testReset() {
  console.log('\n\n########## TESTING RESET ##########');

  // Trigger reset
  resetMission();

  // Wait a bit then log state
  setTimeout(() => {
    logAnimationState('AFTER RESET');

    // Check what trajectory is loaded
    console.log('\n--- Trajectory Analysis ---');
    Object.entries(state.routes || {}).forEach(([did, rd]) => {
      const traj = rd.trajectory || [];
      if (traj.length > 0) {
        // Calculate total distance
        let dist = 0;
        for (let i = 1; i < traj.length; i++) {
          const dx = traj[i][0] - traj[i-1][0];
          const dy = traj[i][1] - traj[i-1][1];
          dist += Math.sqrt(dx*dx + dy*dy);
        }
        console.log(`D${did}: trajLen=${traj.length}, totalDist=${dist.toFixed(1)}`);

        // Check if it matches seg0 from segmentedImport
        if (segmentedImport.isActive()) {
          const seg0Sol = segmentedImport.getSolutionForSegment(0);
          const seg0Traj = seg0Sol?.routes?.[did]?.trajectory || [];
          console.log(`  segmentedImport seg0: ${seg0Traj.length} pts`);
          if (seg0Traj.length > 0) {
            let seg0Dist = 0;
            for (let i = 1; i < seg0Traj.length; i++) {
              const dx = seg0Traj[i][0] - seg0Traj[i-1][0];
              const dy = seg0Traj[i][1] - seg0Traj[i-1][1];
              seg0Dist += Math.sqrt(dx*dx + dy*dy);
            }
            console.log(`  segmentedImport seg0 dist: ${seg0Dist.toFixed(1)}`);
          }
        }

        // Check seg0FullSolution
        if (missionState.seg0FullSolution?.routes?.[did]) {
          const fullTraj = missionState.seg0FullSolution.routes[did].trajectory || [];
          console.log(`  seg0FullSolution: ${fullTraj.length} pts`);
        }
      }
    });
  }, 500);
}

// Test: What happens during animation at segment boundary
function testSegmentSwitch() {
  console.log('\n\n########## TESTING SEGMENT SWITCH ##########');

  // First reset
  resetMission();

  setTimeout(() => {
    logAnimationState('BEFORE ANIMATE');

    // Start animation
    console.log('\n--- Starting Animation ---');
    startAnimation(Object.keys(state.routes).filter(did => state.routes[did]?.trajectory?.length > 0));

    // Monitor for segment switch
    let checkCount = 0;
    const checkInterval = setInterval(() => {
      checkCount++;
      const currentSeg = missionReplay._currentSegmentIndex;
      const missionDist = state.animation.missionDistance;

      console.log(`[Check ${checkCount}] seg=${currentSeg}, missionDist=${missionDist?.toFixed(1)}, animActive=${state.animation.active}`);

      // Log drone states
      Object.entries(state.animation.drones || {}).forEach(([did, ds]) => {
        if (ds.animating !== undefined) {
          console.log(`  D${did}: animating=${ds.animating}, distTraveled=${ds.distanceTraveled?.toFixed(1)}, totalDist=${ds.totalDistance?.toFixed(1)}`);
        }
      });

      // If segment changed, log more details
      if (currentSeg > 0 && checkCount < 50) {
        console.log(`\n!!! SEGMENT SWITCH DETECTED to seg ${currentSeg} !!!`);
        logAnimationState(`AFTER SWITCH TO SEG ${currentSeg}`);
      }

      // Stop after 50 checks or animation stops
      if (checkCount > 100 || !state.animation.active) {
        clearInterval(checkInterval);
        console.log('\n--- Animation Monitoring Complete ---');
        logAnimationState('FINAL STATE');
      }
    }, 500);
  }, 500);
}

// Compare trajectories between segmentedImport and missionReplay
function compareTrajectories() {
  console.log('\n\n########## COMPARING TRAJECTORIES ##########');

  const totalSegs = missionReplay.getSegmentCount();
  console.log(`Total segments: ${totalSegs}`);

  for (let i = 0; i < totalSegs; i++) {
    console.log(`\n--- Segment ${i} ---`);

    // From segmentedImport
    if (segmentedImport.isActive()) {
      const impSol = segmentedImport.getSolutionForSegment(i);
      console.log(`segmentedImport:`);
      Object.entries(impSol?.routes || {}).forEach(([did, rd]) => {
        const traj = rd.trajectory || [];
        if (traj.length > 0) {
          let dist = 0;
          for (let j = 1; j < traj.length; j++) {
            const dx = traj[j][0] - traj[j-1][0];
            const dy = traj[j][1] - traj[j-1][1];
            dist += Math.sqrt(dx*dx + dy*dy);
          }
          console.log(`  D${did}: ${traj.length} pts, dist=${dist.toFixed(1)}, start=[${traj[0][0].toFixed(1)},${traj[0][1].toFixed(1)}], end=[${traj[traj.length-1][0].toFixed(1)},${traj[traj.length-1][1].toFixed(1)}]`);
        }
      });
    }

    // From missionReplay
    const seg = missionReplay.getSegment(i);
    console.log(`missionReplay:`);
    Object.entries(seg?.solution?.routes || {}).forEach(([did, rd]) => {
      const traj = rd.trajectory || [];
      if (traj.length > 0) {
        let dist = 0;
        for (let j = 1; j < traj.length; j++) {
          const dx = traj[j][0] - traj[j-1][0];
          const dy = traj[j][1] - traj[j-1][1];
          dist += Math.sqrt(dx*dx + dy*dy);
        }
        console.log(`  D${did}: ${traj.length} pts, dist=${dist.toFixed(1)}, start=[${traj[0][0].toFixed(1)},${traj[0][1].toFixed(1)}], end=[${traj[traj.length-1][0].toFixed(1)},${traj[traj.length-1][1].toFixed(1)}]`);
      }
    });

    // Cut positions
    const cutPos = segmentedImport.isActive() ? segmentedImport.getCutPositionForSegment(i) : seg?.cutPositions;
    if (cutPos) {
      console.log(`cutPositions:`);
      Object.entries(cutPos).forEach(([did, pos]) => {
        console.log(`  D${did}: [${pos[0].toFixed(1)}, ${pos[1].toFixed(1)}]`);
      });
    }
  }
}

// Run all tests
async function runAllTests() {
  console.log('========================================');
  console.log('ANIMATION COMPARISON TEST');
  console.log('========================================');

  compareTrajectories();

  await wait(1000);

  testReset();

  await wait(2000);

  // testSegmentSwitch();  // Uncomment to test animation
}

// Export functions to window for manual testing
window.testAnimReset = testReset;
window.testAnimSwitch = testSegmentSwitch;
window.testAnimCompare = compareTrajectories;
window.testAnimAll = runAllTests;
window.logAnimState = logAnimationState;

console.log('Animation test functions loaded:');
console.log('  testAnimReset() - Test reset behavior');
console.log('  testAnimSwitch() - Test segment switching during animation');
console.log('  testAnimCompare() - Compare trajectories between sources');
console.log('  testAnimAll() - Run all tests');
console.log('  logAnimState(label) - Log current animation state');
