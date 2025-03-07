FROM mcr.microsoft.com/dotnet/sdk:6.0

WORKDIR /app

COPY receiver.cs .

# Create a new console project and add the source file
RUN dotnet new console -o . --force && \
    mv receiver.cs Program.cs

# Build the project
RUN dotnet build

ENTRYPOINT ["dotnet", "run"]
