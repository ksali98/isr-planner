# ISR Mission Planner

An intelligent web-based mission planning tool for Intelligence, Surveillance, and Reconnaissance (ISR) operations. Design multi-drone missions with AI-powered optimal path planning.

## What is ISR Mission Planner?

ISR Mission Planner is an interactive web application that helps you plan complex multi-drone missions. The tool allows you to:

- **Design Mission Environments**: Place targets, airports, and threats (SAMs) on a map
- **Configure Multiple Drones**: Set up to 5 drones with different capabilities
- **AI-Powered Planning**: Use advanced AI to generate optimal flight paths that:
  - Maximize target coverage based on priority
  - Avoid threat zones (SAM sites)
  - Optimize fuel consumption
  - Coordinate multiple drones efficiently
- **Interactive Visualization**: Watch animated mission playback
- **Segmented Missions**: Break complex missions into multiple segments with checkpoints

## Getting Started

### Accessing the Planner

**Option 1: Use the Deployed Version**
- Visit the live deployment at your Railway URL (see deployment section)

**Option 2: Run Locally**
1. Clone this repository:
   ```bash
   git clone <your-repo-url>
   cd isr-planner
   git checkout release/v1.0-clean
   ```

2. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

3. Set up environment variables (copy `.env.example` to `.env` and add your API keys):
   ```bash
   ANTHROPIC_API_KEY=sk-ant-xxxxx...
   ```

4. Run the server:
   ```bash
   ./run_planner.sh
   ```

5. Open your browser to `http://localhost:8893`

## How to Use the Planner

### 1. Create Your Mission Environment

**Enable Edit Mode:**
- Click the **Edit** button in the toolbar

**Add Mission Elements:**
- **Targets**: Click "Target" button, then click on the map
  - Select target type (A, B, C, D, E) - different point values
  - Set priority (1-10) - higher priority targets preferred by AI

- **Airports**: Click "Airport" button, then click on the map
  - Airports are where drones start and must return to refuel

- **SAM Sites**: Click "SAM" button, then click on the map
  - Set threat range (5-30 units)
  - Drones will avoid these areas during planning

**Modify Elements:**
- **Move**: Drag any element to reposition it
- **Delete**: Select an element and click the "Del" button
- **Disable Edit Mode**: Click "Edit" button again when done

### 2. Configure Drones

**Basic Configuration:**
1. Click on a drone's base (the airport icon)
2. Edit drone properties:
   - **Fuel**: Total fuel capacity
   - **Fuel per Unit**: Fuel consumption rate
   - **Max Targets**: Maximum targets this drone can visit
   - **Target Types**: Which target types (A, B, C, D, E) this drone can engage

**Quick Sequence Entry:**
- Use the sequence input to manually assign a path: e.g., `A1,T1,T2,T3,A1`
- Select drone from dropdown (D1-D5)
- Click "Apply" to set the sequence

### 3. Run the AI Planner

**Single Segment Mission:**
1. Click **Run Planner** in Mission Control
2. The AI will analyze your environment and generate optimal paths
3. Wait for planning to complete (shows progress)
4. Review the generated plan with fuel and points metrics

**Multi-Segment Mission:**
1. Click **"Segment"** menu to create checkpoints
2. Plan each segment separately
3. Use **Checkpoint Replanning** to continue from a checkpoint with different conditions

### 4. Review and Analyze Results

**View Mission Details:**
- **Fuel Usage**: Total fuel consumed per drone
- **Points Collected**: Points earned from targets (higher priority = more points)
- **Path Visualization**: Color-coded paths for each drone
- **Metrics Panel**: Shows total fuel, points, and efficiency

**Animation:**
- Click **Play** to animate the mission
- Adjust speed with the slider
- Watch drones move along their planned paths

### 5. Save and Load Missions

**Save Mission:**
- Click **File** → **Save Environment**
- Environment saved as JSON file
- Includes all targets, airports, SAMs, and drone configurations

**Load Mission:**
- Click **File** → **Load Environment**
- Select a previously saved JSON file
- Mission environment will be restored

**Export Plan:**
- After planning, export the full solution as JSON
- Includes all paths, fuel calculations, and metrics

## Key Features Explained

### AI Planning Algorithm
The planner uses advanced algorithms to:
- **Priority-Based Target Selection**: Higher priority targets are preferred
- **Fuel Optimization**: Finds paths that maximize targets while staying within fuel limits
- **Threat Avoidance**: Routes around SAM threat zones
- **Multi-Drone Coordination**: Prevents duplicate target assignments

### Segmented Missions
Break long missions into segments with checkpoints:
- Plan each leg separately
- Adjust fuel/constraints mid-mission
- Useful for refueling or changing objectives

### Constraint Management
Configure mission constraints:
- Fuel capacity and consumption rates
- Target type restrictions per drone
- Maximum targets per drone
- SAM avoidance zones

## Tips for Best Results

1. **Start Simple**: Begin with 1-2 drones and a few targets to understand the tool
2. **Set Priorities**: Use priority values (1-10) to guide the AI on which targets matter most
3. **Balance Fuel**: Ensure drones have enough fuel to reach targets and return
4. **SAM Placement**: Use SAMs to create realistic no-fly zones
5. **Use Segments**: For complex missions, use segments to break down the problem
6. **Review Before Running**: Double-check your environment setup before planning

## Troubleshooting

**Planner Won't Run:**
- Ensure at least one drone is configured
- Check that drones have valid airports
- Verify API key is set correctly

**No Valid Plan Found:**
- Reduce number of targets
- Increase drone fuel capacity
- Check if SAMs are blocking all paths
- Lower target priorities to make them optional

**Slow Planning:**
- Large missions (20+ targets) may take 1-2 minutes
- Use segments to break down complex missions
- Reduce number of drones if not needed

## Advanced Features

### Checkpoint Replanning
Continue a mission from a checkpoint with modified parameters:
- Useful for refueling scenarios
- Can change fuel levels mid-mission
- Adjust constraints between segments

### Custom Constraints
Edit JSON directly for advanced constraints:
- Custom fuel formulas
- Target type scoring
- Path optimization parameters

## Need Help?

- **Documentation**: See the `/docs` folder for technical details
- **Deployment**: See [DEPLOYMENT.md](DEPLOYMENT.md) for hosting instructions
- **Issues**: Check configuration files and browser console for errors

## Credits

Powered by:
- AI path planning algorithms
- Claude AI for intelligent optimization
- Modern web technologies for interactive visualization
