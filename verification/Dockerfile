FROM mcr.microsoft.com/playwright:v1.56.1-noble
WORKDIR /verification
RUN npm install --no-audit --no-fund playwright@1.56.1 \
    && apt-get update && apt-get install -y --no-install-recommends ffmpeg \
    && rm -rf /var/lib/apt/lists/*
COPY browser.mjs /verification/browser.mjs
ENTRYPOINT ["node", "/verification/browser.mjs"]
