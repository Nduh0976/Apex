using BenchmarkDotNet.Running;

// Run everything:      dotnet run -c Release --project bench/Apex.Benchmarks -- --filter '*'
// Run one class:       dotnet run -c Release --project bench/Apex.Benchmarks -- --filter '*Gradient*'
BenchmarkSwitcher.FromAssembly(typeof(Program).Assembly).Run(args);