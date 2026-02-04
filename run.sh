#!/bin/bash
# Script to install and run the GNSS exporter

echo "🛰️  GNSS Exporter Setup"
echo "======================"

# Create virtual environment if it doesn't exist
if [ ! -d "venv" ]; then
    echo "📦 Creating virtual environment..."
    python3 -m venv venv
fi

# Activate virtual environment
echo "🔧 Activating virtual environment..."
source venv/bin/activate

# Install dependencies
echo "📥 Installing dependencies..."
pip install -r requirements.txt

# Run the exporter
echo ""
echo "🚀 Starting GNSS exporter..."
echo "   Metrics available at: http://localhost:8000/metrics"
echo "   Press Ctrl+C to stop"
echo ""

python gnss-reader.py
