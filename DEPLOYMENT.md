# ISR Mission Planner - Deployment Guide

This guide covers deploying the ISR Mission Planner to Railway.com with Supabase for database storage.

## Prerequisites

- Git repository with your code
- Railway.com account
- Supabase.com account
- Anthropic API key

## Step 1: Set Up Supabase Database

1. **Create a Supabase Project**
   - Go to [supabase.com](https://supabase.com) and sign in
   - Click "New Project"
   - Choose your organization, name your project, and set a database password
   - Select a region close to your users
   - Wait for the project to be created

2. **Run the Database Schema**
   - In your Supabase project, go to "SQL Editor"
   - Copy the contents of `supabase_schema.sql` from this repository
   - Paste and run the SQL to create the required tables:
     - `agent_memories` - Stores agent learning/corrections
     - `sessions` - Stores user sessions
     - `conversations` - Stores conversation history
     - `mission_plans` - Stores completed mission plans

3. **Get Your Supabase Credentials**
   - Go to "Project Settings" → "API"
   - Copy the "Project URL" (this is your `SUPABASE_URL`)
   - Copy the "anon public" key (this is your `SUPABASE_KEY`)

## Step 2: Deploy to Railway

1. **Create a Railway Account**
   - Go to [railway.app](https://railway.app) and sign in with GitHub

2. **Create a New Project**
   - Click "New Project"
   - Select "Deploy from GitHub repo"
   - Connect your GitHub account if not already connected
   - Select your ISR Mission Planner repository

3. **Configure Environment Variables**

   In Railway, go to your project's "Variables" tab and add:

   ```
   ANTHROPIC_API_KEY=sk-ant-xxxxx...
   SUPABASE_URL=https://your-project.supabase.co
   SUPABASE_KEY=your_supabase_anon_key
   PYTHONPATH=/app
   ```

   Optional variables:
   ```
   OPENAI_API_KEY=sk-xxxxx...  # If using GPT models
   DEFAULT_MODEL=claude-sonnet-4-20250514
   DEBUG=false
   ```

4. **Deploy**
   - Railway will automatically detect the Dockerfile and deploy
   - The `railway.toml` file configures the deployment settings
   - Wait for the deployment to complete

5. **Get Your App URL**
   - Go to the "Settings" tab
   - Under "Domains", click "Generate Domain" or add a custom domain
   - Your app will be accessible at `https://your-app.railway.app`

## Step 3: Verify Deployment

1. **Check Health Endpoint**
   ```bash
   curl https://your-app.railway.app/health
   ```
   Should return: `{"status": "healthy"}`

2. **Test the Application**
   - Open `https://your-app.railway.app` in your browser
   - The ISR Mission Planner UI should load
   - Try creating a simple mission plan to verify everything works

## Configuration Files

### railway.toml
```toml
[build]
builder = "dockerfile"
dockerfilePath = "Dockerfile.railway"

[deploy]
numReplicas = 1
healthcheckPath = "/health"
healthcheckTimeout = 30
restartPolicyType = "ON_FAILURE"
restartPolicyMaxRetries = 3
```

### Dockerfile.railway
Optimized Docker image for Railway that:
- Uses Python 3.11 slim base
- Installs all dependencies including Supabase client
- Sets up proper Python path
- Configures health checks
- Runs uvicorn on the Railway-provided PORT

## Database Tables

### agent_memories
Stores agent corrections and learned instructions:
- `id`: Auto-incrementing primary key
- `content`: The memory content (text)
- `category`: Type of memory (correction, instruction, preference, fact)
- `created_at`: Timestamp

### sessions
Stores user sessions:
- `id`: UUID primary key
- `created_at`: Timestamp
- `updated_at`: Auto-updated timestamp
- `metadata`: JSON for additional session data

### conversations
Stores conversation history:
- `id`: Auto-incrementing primary key
- `session_id`: Foreign key to sessions
- `role`: user/assistant/system
- `content`: Message content
- `created_at`: Timestamp
- `metadata`: JSON for token counts, model info, etc.

### mission_plans
Stores completed mission plans:
- `id`: Auto-incrementing primary key
- `session_id`: Optional foreign key
- `plan_data`: JSON with full plan details
- `total_fuel`: Total fuel consumption
- `total_points`: Total points collected
- `drone_count`: Number of drones used
- `created_at`: Timestamp

## Troubleshooting

### Build Fails
- Check Railway logs for specific errors
- Ensure all dependencies are in `requirements.txt`
- Verify Dockerfile syntax

### App Won't Start
- Check that all required environment variables are set
- Verify the PORT variable is being used correctly
- Check Railway logs for startup errors

### Database Connection Issues
- Verify SUPABASE_URL and SUPABASE_KEY are correct
- Ensure the database tables were created
- Check if your Supabase project is active

### Health Check Fails
- Ensure `/health` endpoint returns a 200 status
- Check that the app binds to `0.0.0.0`
- Verify the PORT environment variable is used

## Local Development with Supabase

For local development with Supabase:

1. Copy `.env.example` to `.env`
2. Fill in your Supabase credentials
3. Run the app:
   ```bash
   source venv/bin/activate
   export PYTHONPATH=/path/to/isr_projects
   python -m uvicorn server.main:app --reload --port 8893
   ```

The app will automatically use Supabase when configured, or fall back to local JSON file storage.

## Cost Considerations

### Railway
- Free tier: $5/month credit
- Hobby plan: $5/month per service
- Pro plan: Usage-based pricing

### Supabase
- Free tier: 500MB database, 1GB file storage
- Pro plan: $25/month for more resources

### Anthropic API
- Pay per token usage
- See [anthropic.com/pricing](https://anthropic.com/pricing) for current rates
