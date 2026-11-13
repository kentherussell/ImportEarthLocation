global using static TestChecks;

// The assertion every test program shares.
internal static class TestChecks
{
    public static void Check(bool succeeded, string reason)
    {
        if (!succeeded)
            throw new Exception(reason);
    }
}
