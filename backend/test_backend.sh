#!/bin/bash
# Quick backend test script

echo "🧪 Testing Backend..."

# Test 1: Health check
echo ""
echo "1. Testing health endpoint..."
curl -s --max-time 5 http://localhost:8000/health && echo " ✅" || echo " ❌ Failed"

# Test 2: Root endpoint
echo ""
echo "2. Testing root endpoint..."
curl -s --max-time 5 http://localhost:8000/ && echo " ✅" || echo " ❌ Failed"

# Test 3: Chat query (this will take 5-15 seconds)
echo ""
echo "3. Testing chat query (this may take 10-20 seconds)..."
curl -X POST http://localhost:8000/api/chat/query \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS 2?", "user_id": "test"}' \
  --max-time 30 \
  -s | python3 -m json.tool 2>/dev/null | head -10 || echo "Response received (may have timed out)"

echo ""
echo "✅ Backend test complete!"

