FROM node:24-alpine@sha256:a34ab6d7f6f4813f4d0c5469d74685604b962fbdbf5793a7bc7b4998ecb867c8
WORKDIR /app
COPY package.json package-lock.json ./
RUN npm ci --omit=dev --ignore-scripts && npm cache clean --force
COPY server.mjs bridge.mjs setting_bridge.mjs drive_bridge.mjs tools_bridge.mjs VOICE_RULES.md oauth.mjs oauth_language.mjs catalog.mjs catalog.json ./
RUN chmod 644 /app/*.mjs /app/*.json /app/*.md && mkdir /app/state && chown node:node /app/state
USER node
RUN node -e "import('./server.mjs').then(m => m.createApp())"
EXPOSE 18770
CMD ["node", "server.mjs"]
