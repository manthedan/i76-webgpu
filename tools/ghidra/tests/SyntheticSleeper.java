/* Self-authored unrelated JVM used to prove timeout cleanup is process-scoped. */
public final class SyntheticSleeper {
    private SyntheticSleeper() {}

    public static void main(String[] args) throws InterruptedException {
        Thread.sleep(30_000L);
    }
}
